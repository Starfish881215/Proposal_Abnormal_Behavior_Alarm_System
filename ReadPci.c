#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/BaseLib.h>
#include <Library/IoLib.h>
#include <Protocol/AcpiSystemDescriptionTable.h>
#include <IndustryStandard/Acpi.h>

#pragma pack(1)
typedef struct {
  EFI_ACPI_DESCRIPTION_HEADER Header;
  UINT64 Reserved;
} MCFG_TABLE;

typedef struct {
  UINT64 BaseAddress;
  UINT16 PciSegmentGroupNumber;
  UINT8 StartBusNumber;
  UINT8 EndBusNumber;
  UINT32 Reserved;
} MCFG_ALLOCATION;
#pragma pack()

static MCFG_ALLOCATION *McfgAllocs;
static UINTN McfgAllocCount;
static BOOLEAN McfgReady;
static UINT8 *VpdBuf;
static UINTN VpdBufCap;

static UINTN CurSeg;
static UINTN CurBus;
static UINTN CurDev;
static UINTN CurFunc;
static BOOLEAN CurLocValid;

enum {
  OFF_VID = 0x00,
  OFF_DID = 0x02,
  OFF_CMD = 0x04,
  OFF_STS = 0x06,
  OFF_PROGIF = 0x09,
  OFF_SUB = 0x0A,
  OFF_CLASS = 0x0B,
  OFF_HDR = 0x0E,
  OFF_BAR0 = 0x10,
  OFF_BAR1 = 0x14,
  OFF_PBUS = 0x18,
  OFF_SBUS = 0x19,
  OFF_UBUS = 0x1A,
  OFF_CAP_PTR = 0x34,
  OFF_CAP_PTR_CB = 0x14,
  OFF_BR_IOB = 0x1C,
  OFF_BR_IOL = 0x1D,
  OFF_BR_MEMB = 0x20,
  OFF_BR_MEML = 0x22,
  OFF_BR_PREFB = 0x24,
  OFF_BR_PREFL = 0x26,
  OFF_BR_PREFB_UP = 0x28,
  OFF_BR_PREFL_UP = 0x2C,
  OFF_BR_IOB_UP = 0x30,
  OFF_BR_IOL_UP = 0x32
};

enum { STS_CAP_LIST = BIT4, CMD_IO = BIT0, CMD_MEM = BIT1 };

typedef enum { BarUnused = 0, BarIo, BarMem32, BarMem64 } BAR_TYPE;

typedef struct {
  UINTN Seg;
  UINTN Bus;
  UINTN Dev;
  UINTN Func;
  UINT8 Pri;
  UINT8 Sec;
  UINT8 Sub;
  struct {
    UINT32 RawLo;
    UINT32 RawHi;
    BAR_TYPE Type;
    BOOLEAN Pref;
    UINT64 Base;
    UINT64 Size;
    UINT64 End;
    UINTN Dw;
    EFI_STATUS St;
  } B0, B1;
  struct {
    UINT16 MemBaseRaw;
    UINT16 MemLimitRaw;
    UINT8 IoBaseRaw;
    UINT8 IoLimitRaw;
    UINT16 IoBaseUp;
    UINT16 IoLimitUp;
    UINT32 MemBase;
    UINT32 MemLimit;
    UINT32 MemSize;
    BOOLEAN MemEn;
    UINT32 IoBase;
    UINT32 IoLimit;
    UINT32 IoSize;
    BOOLEAN IoEn;
    BOOLEAN Io32;
    UINT16 PrefBaseRaw;
    UINT16 PrefLimitRaw;
    UINT32 PrefBaseUp;
    UINT32 PrefLimitUp;
  } W;
} BRIDGE_INFO;

/* ---------- small lookup tables ---------- */
typedef struct {
  UINT16 Id;
  CHAR16 *Name;
} IDN;

static IDN gStdCaps[] = {
  {0x01, L"Power Management"},
  {0x03, L"VPD"},
  {0x05, L"MSI"},
  {0x07, L"PCI-X"},
  {0x09, L"Vendor Specific"},
  {0x10, L"PCI Express"},
  {0x11, L"MSI-X"}
};

static IDN gExtCaps[] = {
  {0x0001, L"AER"},
  {0x0003, L"Device Serial Number"},
  {0x000B, L"Vendor-Specific Extended"},
  {0x0010, L"ACS"},
  {0x001E, L"Resizable BAR"},
  {0x0026, L"L1 PM Substates"}
};

static CHAR16 *IdName(IDN *T, UINTN N, UINT16 Id) {
  for (UINTN i = 0; i < N; i++) {
    if (T[i].Id == Id) {
      return T[i].Name;
    }
  }
  return L"Unknown";
}

static CHAR16 *PcieTypeStr(UINT8 T) {
  switch (T) {
    case 0x0:
      return L"PCIe Endpoint";
    case 0x1:
      return L"Legacy PCIe Endpoint";
    case 0x4:
      return L"Root Port";
    case 0x5:
      return L"Upstream Port";
    case 0x6:
      return L"Downstream Port";
    case 0x7:
      return L"PCIe-to-PCI/PCI-X Bridge";
    case 0x8:
      return L"PCI/PCI-X-to-PCIe Bridge";
    case 0x9:
      return L"RC Integrated Endpoint";
    case 0xA:
      return L"RC Event Collector";
    default:
      return L"Unknown Type";
  }
}

static CHAR16 *BarStr(BAR_TYPE T) {
  return (T == BarIo) ? L"IO" : (T == BarMem32) ? L"MEM32" : (T == BarMem64) ? L"MEM64" : L"UNUSED";
}

/* ---------- MCFG / ECAM ---------- */
static EFI_STATUS InitMcfg(VOID) {
  if (McfgReady) {
    return EFI_SUCCESS;
  }

  EFI_ACPI_SDT_PROTOCOL *Sdt = NULL;
  EFI_STATUS St = gBS->LocateProtocol(&gEfiAcpiSdtProtocolGuid, NULL, (VOID **)&Sdt);
  if (EFI_ERROR(St) || !Sdt) {
    return St;
  }

  UINTN Idx = 0;
  UINTN Key = 0;
  EFI_ACPI_SDT_HEADER *T = NULL;
  EFI_ACPI_TABLE_VERSION Ver;
  while (TRUE) {
    St = Sdt->GetAcpiTable(Idx++, &T, &Ver, &Key);
    if (EFI_ERROR(St) || !T) {
      break;
    }

    if (T->Signature == SIGNATURE_32('M', 'C', 'F', 'G')) {
      MCFG_TABLE *M = (MCFG_TABLE *)T;
      if (M->Header.Length < sizeof(MCFG_TABLE)) {
        return EFI_COMPROMISED_DATA;
      }

      UINTN Bytes = M->Header.Length - sizeof(MCFG_TABLE);
      UINTN Cnt = Bytes / sizeof(MCFG_ALLOCATION);
      if (!Cnt) {
        return EFI_NOT_FOUND;
      }

      McfgAllocs = AllocateZeroPool(sizeof(MCFG_ALLOCATION) * Cnt);
      if (!McfgAllocs) {
        return EFI_OUT_OF_RESOURCES;
      }

      CopyMem(McfgAllocs, (UINT8 *)M + sizeof(MCFG_TABLE), sizeof(MCFG_ALLOCATION) * Cnt);
      McfgAllocCount = Cnt;
      McfgReady = TRUE;
      return EFI_SUCCESS;
    }
  }
  return EFI_NOT_FOUND;
}

static MCFG_ALLOCATION *FindMcfg(UINTN Seg, UINTN Bus) {
  if (!McfgReady || !McfgAllocs || !McfgAllocCount) {
    return NULL;
  }
  for (UINTN i = 0; i < McfgAllocCount; i++) {
    if (McfgAllocs[i].PciSegmentGroupNumber == (UINT16)Seg &&
        Bus >= McfgAllocs[i].StartBusNumber &&
        Bus <= McfgAllocs[i].EndBusNumber) {
      return &McfgAllocs[i];
    }
  }
  return NULL;
}

static VOID SetCurrentLocation(UINTN Seg, UINTN Bus, UINTN Dev, UINTN Func) {
  CurSeg = Seg;
  CurBus = Bus;
  CurDev = Dev;
  CurFunc = Func;
  CurLocValid = TRUE;
}

static EFI_STATUS CfgAddr(UINT32 Off, UINTN *A) {
  if (!A) {
    return EFI_INVALID_PARAMETER;
  }

  if (!CurLocValid) {
    return EFI_NOT_READY;
  }

  EFI_STATUS St = InitMcfg();
  if (EFI_ERROR(St)) {
    return St;
  }

  MCFG_ALLOCATION *M = FindMcfg(CurSeg, CurBus);
  if (!M) {
    return EFI_NOT_FOUND;
  }

  UINT64 BusOff = (UINT64)(CurBus - M->StartBusNumber);
  *A = (UINTN)(M->BaseAddress + (BusOff << 20) + ((UINT64)CurDev << 15) + ((UINT64)CurFunc << 12) + (UINT64)Off);
  return EFI_SUCCESS;
}

static EFI_STATUS CfgRd(UINTN W, UINT32 Off, VOID *V) {
  if (!V) {
    return EFI_INVALID_PARAMETER;
  }

  UINTN A;
  EFI_STATUS St = CfgAddr(Off, &A);
  if (EFI_ERROR(St)) {
    return St;
  }

  if (W == 1) {
    *(UINT8 *)V = MmioRead8(A);
    return EFI_SUCCESS;
  }
  if (W == 2) {
    *(UINT16 *)V = MmioRead16(A);
    return EFI_SUCCESS;
  }
  *(UINT32 *)V = MmioRead32(A);
  return EFI_SUCCESS;
}

static EFI_STATUS CfgWr(UINTN W, UINT32 Off, UINTN Val) {
  UINTN A;
  EFI_STATUS St = CfgAddr(Off, &A);
  if (EFI_ERROR(St)) {
    return St;
  }

  if (W == 2) {
    MmioWrite16(A, (UINT16)Val);
    return EFI_SUCCESS;
  }
  MmioWrite32(A, (UINT32)Val);
  return EFI_SUCCESS;
}

#define R8(o, p) CfgRd(1, (o), (p))
#define R16(o, p) CfgRd(2, (o), (p))
#define R32(o, p) CfgRd(4, (o), (p))
#define W16(o, v) CfgWr(2, (o), (v))
#define W32(o, v) CfgWr(4, (o), (v))

static VOID DumpMcfg(VOID) {
  EFI_STATUS St = InitMcfg();
  Print(L"\n============================================================\n");
  if (EFI_ERROR(St)) {
    Print(L"[MCFG] Init failed: %r\n", St);
    return;
  }
  Print(L"[MCFG] Cached EntryCount: %u\n", McfgAllocCount);
  for (UINTN i = 0; i < McfgAllocCount; i++) {
    Print(L"  [Entry %u]\n", i);
    Print(L"    BaseAddress : 0x%016lx\n", McfgAllocs[i].BaseAddress);
    Print(L"    Segment     : %u\n", McfgAllocs[i].PciSegmentGroupNumber);
    Print(L"    BusRange    : %u ~ %u\n", McfgAllocs[i].StartBusNumber, McfgAllocs[i].EndBusNumber);
    Print(L"    Reserved    : 0x%08x\n", McfgAllocs[i].Reserved);
  }
}

/* ---------- VPD ---------- */
static BOOLEAN IsAscii(UINT8 b) { return (b >= 0x20 && b <= 0x7E); }

static VOID HexAscii(UINT8 *B, UINTN L) {
  for (UINTN o = 0; o < L; o += 16) {
    Print(L"%03x: ", o);
    for (UINTN i = 0; i < 16; i++) {
      if (o + i < L) {
        Print(L"%02x ", B[o + i]);
      } else {
        Print(L"   ");
      }
    }
    Print(L" |");
    for (UINTN i = 0; i < 16 && o + i < L; i++) {
      UINT8 c = B[o + i];
      Print(L"%c", IsAscii(c) ? (CHAR16)c : L'.');
    }
    Print(L"|\n");
  }
}

static VOID VpdStr(UINT8 *B, UINTN L) {
  if (!B || !L) {
    Print(L"<empty>");
    return;
  }
  for (UINTN i = 0; i < L; i++) {
    UINT8 c = B[i];
    if (c == '\\') {
      Print(L"\\\\");
      continue;
    }
    if (c == 0x00 && i == L - 1) {
      continue;
    }
    if (!IsAscii(c)) {
      Print(L"\\x%02x", c);
    } else {
      Print(L"%c", (CHAR16)c);
    }
  }
}

static CHAR16 *VpdKey(UINT8 A, UINT8 B) {
  if (A == 'P' && B == 'N') {
    return L"Part number";
  }
  if (A == 'S' && B == 'N') {
    return L"Serial number";
  }
  if (A == 'E' && B == 'C') {
    return L"Engineering changes";
  }
  if (A == 'M' && B == 'N') {
    return L"Manufacturer ID";
  }
  if (A == 'F' && B == 'R') {
    return L"FRU number";
  }
  if (A == 'R' && B == 'V') {
    return L"Reserved";
  }
  if (A == 'V') {
    return L"Vendor specific";
  }
  if (A == 'Y') {
    return L"System specific";
  }
  return L"Unknown";
}

static EFI_STATUS VpdRdD(UINT8 Cap, UINT16 Addr, UINT32 *D) {
  if (!D) {
    return EFI_INVALID_PARAMETER;
  }

  UINT16 A = (UINT16)(Addr & 0xFFFC);
  UINT16 Req = (UINT16)(A & 0x7FFF);
  EFI_STATUS St = W16((UINT32)Cap + 0x02, Req);
  if (EFI_ERROR(St)) {
    return St;
  }

  for (UINTN i = 0; i < 2000; i++) {
    UINT16 Cur = 0;
    St = R16((UINT32)Cap + 0x02, &Cur);
    if (EFI_ERROR(St)) {
      return St;
    }
    if (Cur & 0x8000) {
      return R32((UINT32)Cap + 0x04, D);
    }
    gBS->Stall(100);
  }
  return EFI_TIMEOUT;
}

static EFI_STATUS VpdRdB(UINT8 Cap, UINT16 Addr, UINT8 *B) {
  if (!B) {
    return EFI_INVALID_PARAMETER;
  }
  UINT32 D = 0;
  EFI_STATUS St = VpdRdD(Cap, Addr, &D);
  if (EFI_ERROR(St)) {
    return St;
  }
  *B = (UINT8)((D >> ((Addr & 3) * 8)) & 0xFF);
  return EFI_SUCCESS;
}

static EFI_STATUS VpdRaw(UINT8 Cap, UINT8 **Out, UINTN *OutLen) {
  if (!Out || !OutLen) {
    return EFI_INVALID_PARAMETER;
  }
  *Out = NULL;
  *OutLen = 0;

  if (!VpdBuf) {
    VpdBufCap = 1024;
    VpdBuf = AllocateZeroPool(VpdBufCap);
    if (!VpdBuf) {
      return EFI_OUT_OF_RESOURCES;
    }
  } else {
    ZeroMem(VpdBuf, VpdBufCap);
  }

  EFI_STATUS St;
  UINTN L = 0;
  while (TRUE) {
    UINT8 b = 0;
    St = VpdRdB(Cap, (UINT16)L, &b);
    if (EFI_ERROR(St)) {
      Print(L"    [VPD] Read failed at 0x%03x: %r\n", L, St);
      return St;
    }

    if (L >= VpdBufCap) {
      UINTN NewCap = VpdBufCap * 2;
      if (NewCap > 0x8000) {
        NewCap = 0x8000;
      }
      if (NewCap <= VpdBufCap) {
        Print(L"    [VPD] Buffer cap reached without EndTag (cap=%u)\n", VpdBufCap);
        break;
      }
      UINT8 *N = AllocateZeroPool(NewCap);
      if (!N) {
        return EFI_OUT_OF_RESOURCES;
      }
      CopyMem(N, VpdBuf, VpdBufCap);
      FreePool(VpdBuf);
      VpdBuf = N;
      VpdBufCap = NewCap;
    }

    VpdBuf[L++] = b;
    if (b == 0x78) {
      break;
    }
    if (L >= 0x8000) {
      Print(L"    [VPD] No EndTag found within 0x8000 bytes\n");
      break;
    }
  }

  *Out = VpdBuf;
  *OutLen = L;
  return EFI_SUCCESS;
}

static VOID VpdParse(UINT8 *Raw, UINTN RawLen) {
  if (!Raw || !RawLen) {
    Print(L"    <empty VPD>\n");
    return;
  }
  UINTN Off = 0;
  UINT8 Csum = 0;

  while (Off < RawLen) {
    UINT16 Res = (UINT16)Off;
    UINT8 Tag = Raw[Off];
    Csum = (UINT8)(Csum + Tag);
    if (Tag == 0x78) {
      Print(L"    End\n");
      break;
    }

    if (Tag & 0x80) {
      if (Off + 2 >= RawLen) {
        break;
      }

      UINT8 Lo = Raw[Off + 1];
      UINT8 Hi = Raw[Off + 2];
      Csum = (UINT8)(Csum + Lo);
      Csum = (UINT8)(Csum + Hi);

      UINT16 Len = (UINT16)(Lo | (Hi << 8));
      UINT16 DataOff = (UINT16)(Off + 3);
      UINT8 Type = (UINT8)(Tag & 0x7F);
      Print(L"    [VPD-RES] @0x%04x Tag=0x%02x Type=0x%02x Len=0x%04x DataOff=0x%04x (Csum=0x%02x)\n",
            Res, Tag, Type, Len, DataOff, Csum);

      if ((UINTN)DataOff + Len > RawLen) {
        Print(L"      [VPD-RES] length overruns raw buffer, stop.\n");
        break;
      }

      if (Tag == 0x82) {
        Print(L"    Product Name: ");
        for (UINT16 i = 0; i < Len; i++) {
          Csum = (UINT8)(Csum + Raw[DataOff + i]);
        }
        VpdStr(&Raw[DataOff], Len);
        Print(L"\n");
      } else if (Tag == 0x90 || Tag == 0x91) {
        BOOLEAN Ro = (Tag == 0x90);
        Print(L"    VPD %s fields:\n", Ro ? L"Read-only" : L"Read-write");
        UINT16 p = 0;
        while ((UINT16)(p + 3) <= Len) {
          UINT8 K0 = Raw[DataOff + p];
          UINT8 K1 = Raw[DataOff + p + 1];
          UINT8 VLen = Raw[DataOff + p + 2];
          Csum = (UINT8)(Csum + K0);
          Csum = (UINT8)(Csum + K1);
          Csum = (UINT8)(Csum + VLen);
          UINT16 ValOff = (UINT16)(DataOff + p + 3);

          if ((UINT16)(p + 3 + VLen) > Len) {
            Print(L"      [VPD] KV overruns resource, stop.\n");
            break;
          }

          if (Ro && K0 == 'R' && K1 == 'V') {
            UINT8 Check = (VLen >= 1) ? Raw[ValOff] : 0;
            if (VLen >= 1) {
              Csum = (UINT8)(Csum + Check);
            }
            UINTN Rsv = (VLen >= 1) ? (UINTN)(VLen - 1) : 0;
            Print(L"      @0x%03x [RV] Reserved: checksum %s, %u byte(s) reserved", ValOff,
                  (Csum == 0) ? L"good" : L"bad", Rsv);
            if (Csum != 0) {
              Print(L", (running csum=0x%02x, RVbyte=0x%02x)", Csum, Check);
            }
            Print(L"\n");
          } else {
            for (UINT8 i = 0; i < VLen; i++) {
              Csum = (UINT8)(Csum + Raw[ValOff + i]);
            }
            Print(L"      @0x%03x [%c%c] %s: ", ValOff, (CHAR16)K0, (CHAR16)K1, VpdKey(K0, K1));
            VpdStr(&Raw[ValOff], VLen);
            Print(L"\n");
          }

          p = (UINT16)(p + 3 + VLen);
        }
      } else {
        for (UINT16 i = 0; i < Len; i++) {
          Csum = (UINT8)(Csum + Raw[DataOff + i]);
        }
      }

      Off = (UINTN)(DataOff + Len);
    } else {
      UINT8 sLen = (UINT8)(Tag & 0x07);
      UINT8 sType = (UINT8)(Tag >> 3);
      UINT16 DataOff = (UINT16)(Off + 1);
      Print(L"    [VPD-RES] @0x%04x Tag=0x%02x SmallType=0x%02x Len=%u DataOff=0x%04x (Csum=0x%02x)\n",
            Res, Tag, sType, sLen, DataOff, Csum);

      if ((UINTN)DataOff + sLen > RawLen) {
        break;
      }
      for (UINT8 i = 0; i < sLen; i++) {
        Csum = (UINT8)(Csum + Raw[DataOff + i]);
      }
      Off = (UINTN)(DataOff + sLen);
    }
  }
}

static EFI_STATUS DumpVpd(UINT8 Cap) {
  Print(L"  [VPD] Capability @ 0x%02x\n", Cap);
  UINT8 *Raw = NULL;
  UINTN L = 0;
  EFI_STATUS St = VpdRaw(Cap, &Raw, &L);
  if (EFI_ERROR(St)) {
    return St;
  }
  Print(L"    [VPD-RAW] Dump (EndTag=0x78), %u bytes shown:\n", L);
  HexAscii(Raw, L);
  Print(L"\n");
  VpdParse(Raw, L);
  return EFI_SUCCESS;
}

/* ---------- Caps ---------- */
static UINT8 DumpStdCaps(UINT8 HdrType) {
  UINT16 St = 0;
  if (EFI_ERROR(R16(OFF_STS, &St))) {
    Print(L"  [CAP] Read Status failed\n");
    return 0;
  }
  if ((St & STS_CAP_LIST) == 0) {
    Print(L"  [CAP] Standard Capabilities: <none>\n");
    return 0;
  }

  UINT8 PtrOff = (HdrType == 0x02) ? OFF_CAP_PTR_CB : OFF_CAP_PTR;
  UINT8 P = 0;
  if (EFI_ERROR(R8(PtrOff, &P))) {
    Print(L"  [CAP] Read CapPtr failed\n");
    return 0;
  }

  P &= 0xFC;
  if (!P) {
    Print(L"  [CAP] Standard Capabilities: <cap bit set but ptr=0>\n");
    return 0;
  }

  BOOLEAN Vis[256];
  ZeroMem(Vis, sizeof(Vis));
  UINT8 VpdOff = 0;

  Print(L"  [CAP] Standard Capabilities (ptr=0x%02x):\n", P);
  for (UINTN g = 0; g < 64 && P; g++) {
    if (Vis[P]) {
      Print(L"    [0x%02x] <loop detected, stop>\n", P);
      break;
    }
    Vis[P] = TRUE;

    UINT8 Id = 0;
    UINT8 N = 0;
    if (EFI_ERROR(R8(P + 0, &Id)) || EFI_ERROR(R8(P + 1, &N))) {
      Print(L"    [0x%02x] <read failed>\n", P);
      break;
    }
    if (Id == 0x03 && !VpdOff) {
      VpdOff = P;
    }

    if (Id == 0x10) {
      UINT16 Pc = 0;
      if (!EFI_ERROR(R16(P + 2, &Pc))) {
        UINT8 Ver = (UINT8)(Pc & 0x0F);
        UINT8 Type = (UINT8)((Pc >> 4) & 0x0F);
        Print(L"    [0x%02x] CapID=0x%02x (%s)  Next=0x%02x  (v%u, %s)\n", P, Id,
              IdName(gStdCaps, ARRAY_SIZE(gStdCaps), Id), (UINT8)(N & 0xFC), Ver, PcieTypeStr(Type));
      } else {
        Print(L"    [0x%02x] CapID=0x%02x (%s)  Next=0x%02x\n", P, Id,
              IdName(gStdCaps, ARRAY_SIZE(gStdCaps), Id), (UINT8)(N & 0xFC));
      }
    } else {
      Print(L"    [0x%02x] CapID=0x%02x (%s)  Next=0x%02x\n", P, Id,
            IdName(gStdCaps, ARRAY_SIZE(gStdCaps), Id), (UINT8)(N & 0xFC));
    }

    P = (UINT8)(N & 0xFC);
  }
  return VpdOff;
}

static VOID DumpExtCaps(VOID) {
  UINT32 H = 0;
  if (EFI_ERROR(R32(0x100, &H))) {
    Print(L"  [EXT] Extended Capabilities: <read failed>\n");
    return;
  }
  if (H == 0 || H == 0xFFFFFFFF) {
    Print(L"  [EXT] Extended Capabilities: <none>\n");
    return;
  }

  BOOLEAN Vis[4096];
  ZeroMem(Vis, sizeof(Vis));
  UINT32 Off = 0x100;

  Print(L"  [EXT] PCIe Extended Capabilities:\n");
  for (UINTN g = 0; g < 128 && Off >= 0x100 && Off < 0x1000; g++) {
    if (Vis[Off]) {
      Print(L"    [0x%03x] <loop detected, stop>\n", Off);
      break;
    }
    Vis[Off] = TRUE;

    if (EFI_ERROR(R32(Off, &H))) {
      Print(L"    [0x%03x] <read failed>\n", Off);
      break;
    }
    if (H == 0 || H == 0xFFFFFFFF) {
      break;
    }

    UINT16 Id = (UINT16)(H & 0xFFFF);
    UINT8 Ver = (UINT8)((H >> 16) & 0x0F);
    UINT16 NextDw = (UINT16)((H >> 20) & 0x0FFF);
    UINT32 Next = ((UINT32)NextDw) << 2;

    Print(L"    [0x%03x] ExtCapID=0x%04x (%s)  Ver=%u  Next=0x%03x\n", Off, Id,
          IdName(gExtCaps, ARRAY_SIZE(gExtCaps), Id), Ver, Next);
    if (!Next) {
      break;
    }
    Off = Next;
  }
}

/* ---------- Bridge ---------- */
static VOID DecMem(UINT16 B, UINT16 L, UINT32 *Base, UINT32 *Lim, BOOLEAN *En) {
  UINT32 b = ((UINT32)(B & 0xFFF0)) << 16;
  UINT32 l = (((UINT32)(L & 0xFFF0)) << 16) | 0xFFFFF;
  *Base = b;
  *Lim = l;
  *En = (b <= l);
}

static VOID DecIo(UINT8 B, UINT8 L, UINT16 Bu, UINT16 Lu, UINT32 *Base, UINT32 *Lim, BOOLEAN *Is32, BOOLEAN *En) {
  UINT8 Bt = (UINT8)(B & 0x0F);
  UINT8 Lt = (UINT8)(L & 0x0F);
  *Is32 = (Bt == 0x01) && (Lt == 0x01);
  UINT32 bLo = ((UINT32)(B & 0xF0)) << 8;
  UINT32 lLo = (((UINT32)(L & 0xF0)) << 8) | 0xFFF;
  UINT32 b = *Is32 ? ((((UINT32)Bu) << 16) | bLo) : bLo;
  UINT32 l = *Is32 ? ((((UINT32)Lu) << 16) | lLo) : lLo;
  *Base = b;
  *Lim = l;
  *En = (b <= l);
}

static EFI_STATUS ProbeBar(UINT32 Off, BAR_TYPE *Type, BOOLEAN *Pref, UINT64 *Base, UINT64 *Size, UINTN *Dw) {
  EFI_STATUS St;
  UINT32 OLo = 0;
  UINT32 OHi = 0;
  UINT32 MLo = 0;
  UINT32 MHi = 0;
  UINT16 Cmd0 = 0;
  UINT16 Cmd1 = 0;
  *Type = BarUnused;
  *Pref = FALSE;
  *Base = 0;
  *Size = 0;
  *Dw = 1;

  St = R32(Off, &OLo);
  if (EFI_ERROR(St)) {
    return St;
  }
  St = R16(OFF_CMD, &Cmd0);
  if (EFI_ERROR(St)) {
    return St;
  }

  Cmd1 = (UINT16)(Cmd0 & ~(CMD_IO | CMD_MEM));
  St = W16(OFF_CMD, Cmd1);
  if (EFI_ERROR(St)) {
    return St;
  }

  if (OLo & BIT0) {
    *Type = BarIo;
    *Dw = 1;
    St = W32(Off, 0xFFFFFFFF);
    if (EFI_ERROR(St)) {
      goto Done;
    }
    St = R32(Off, &MLo);
    if (EFI_ERROR(St)) {
      goto Done;
    }
    St = W32(Off, OLo);
    if (EFI_ERROR(St)) {
      goto Done;
    }
    {
      UINT32 b = OLo & 0xFFFFFFFC;
      UINT32 m = MLo & 0xFFFFFFFC;
      if (m) {
        *Base = (UINT64)b;
        *Size = (UINT64)((~m) + 1);
      }
    }
  } else {
    UINT32 mt = (OLo >> 1) & 0x3;
    *Pref = (OLo & BIT3) ? TRUE : FALSE;
    if (mt == 0x2) {
      *Type = BarMem64;
      *Dw = 2;
      St = R32(Off + 4, &OHi);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      St = W32(Off, 0xFFFFFFFF);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      St = W32(Off + 4, 0xFFFFFFFF);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      St = R32(Off, &MLo);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      St = R32(Off + 4, &MHi);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      St = W32(Off, OLo);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      St = W32(Off + 4, OHi);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      {
        UINT64 b = (((UINT64)OHi) << 32) | (UINT64)(OLo & 0xFFFFFFF0);
        UINT64 m = (((UINT64)MHi) << 32) | (UINT64)(MLo & 0xFFFFFFF0);
        if (m) {
          *Base = b;
          *Size = (~m) + 1;
        }
      }
    } else {
      *Type = BarMem32;
      *Dw = 1;
      St = W32(Off, 0xFFFFFFFF);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      St = R32(Off, &MLo);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      St = W32(Off, OLo);
      if (EFI_ERROR(St)) {
        goto Done;
      }
      {
        UINT32 b = OLo & 0xFFFFFFF0;
        UINT32 m = MLo & 0xFFFFFFF0;
        if (m) {
          *Base = (UINT64)b;
          *Size = (UINT64)((~m) + 1);
        }
      }
    }
  }

Done:
  W16(OFF_CMD, Cmd0);
  return St;
}

static EFI_STATUS CollectBridge(UINTN Seg, UINTN Bus, UINTN Dev, UINTN Func, BRIDGE_INFO *I) {
  if (!I) {
    return EFI_INVALID_PARAMETER;
  }
  ZeroMem(I, sizeof(*I));

  I->Seg = Seg;
  I->Bus = Bus;
  I->Dev = Dev;
  I->Func = Func;

  R8(OFF_PBUS, &I->Pri);
  R8(OFF_SBUS, &I->Sec);
  R8(OFF_UBUS, &I->Sub);
  R32(OFF_BAR0, &I->B0.RawLo);
  R32(OFF_BAR1, &I->B1.RawLo);

  I->B0.St = ProbeBar(OFF_BAR0, &I->B0.Type, &I->B0.Pref, &I->B0.Base, &I->B0.Size, &I->B0.Dw);
  if (!EFI_ERROR(I->B0.St) && I->B0.Size) {
    I->B0.End = I->B0.Base + I->B0.Size - 1;
  }

  if (I->B0.Type == BarMem64) {
    I->B0.RawHi = I->B1.RawLo;
    I->B1.Type = BarUnused;
    I->B1.Size = 0;
  } else {
    I->B1.St = ProbeBar(OFF_BAR1, &I->B1.Type, &I->B1.Pref, &I->B1.Base, &I->B1.Size, &I->B1.Dw);
    if (!EFI_ERROR(I->B1.St) && I->B1.Size) {
      I->B1.End = I->B1.Base + I->B1.Size - 1;
    }
  }

  R16(OFF_BR_MEMB, &I->W.MemBaseRaw);
  R16(OFF_BR_MEML, &I->W.MemLimitRaw);
  R8(OFF_BR_IOB, &I->W.IoBaseRaw);
  R8(OFF_BR_IOL, &I->W.IoLimitRaw);
  R16(OFF_BR_IOB_UP, &I->W.IoBaseUp);
  R16(OFF_BR_IOL_UP, &I->W.IoLimitUp);

  DecMem(I->W.MemBaseRaw, I->W.MemLimitRaw, &I->W.MemBase, &I->W.MemLimit, &I->W.MemEn);
  if (I->W.MemEn) {
    I->W.MemSize = I->W.MemLimit - I->W.MemBase + 1;
  }

  DecIo(I->W.IoBaseRaw, I->W.IoLimitRaw, I->W.IoBaseUp, I->W.IoLimitUp, &I->W.IoBase, &I->W.IoLimit,
        &I->W.Io32, &I->W.IoEn);
  if (I->W.IoEn) {
    I->W.IoSize = I->W.IoLimit - I->W.IoBase + 1;
  }

  R16(OFF_BR_PREFB, &I->W.PrefBaseRaw);
  R16(OFF_BR_PREFL, &I->W.PrefLimitRaw);
  R32(OFF_BR_PREFB_UP, &I->W.PrefBaseUp);
  R32(OFF_BR_PREFL_UP, &I->W.PrefLimitUp);
  return EFI_SUCCESS;
}

static VOID PrintBridge(BRIDGE_INFO *I, UINTN Idx) {
  Print(L"\n============================================================\n");
  Print(L"Bridge #%u @ %04x:%02x:%02x.%x  (Pri=%02x Sec=%02x Sub=%02x)\n", Idx, I->Seg, I->Bus, I->Dev, I->Func, I->Pri,
        I->Sec, I->Sub);

  if (I->B0.Type == BarMem64) {
    Print(L"  [RAW] BAR0: LO=0x%08x HI=0x%08x\n", I->B0.RawLo, I->B0.RawHi);
  } else {
    Print(L"  [RAW] BAR0: 0x%08x\n", I->B0.RawLo);
  }

  if (EFI_ERROR(I->B0.St)) {
    Print(L"  [BAR] BAR0: probe failed: %r\n", I->B0.St);
  } else if (!I->B0.Size) {
    Print(L"  [BAR] BAR0: <unused or size=0>\n");
  } else {
    Print(L"  [BAR] BAR0: %s%s  Base=0x%016lx  Size=0x%016lx  End=0x%016lx\n", BarStr(I->B0.Type),
          I->B0.Pref ? L"(P)" : L"", I->B0.Base, I->B0.Size, I->B0.End);
  }

  if (I->B0.Type == BarMem64) {
    Print(L"  [RAW] BAR1: <consumed as BAR0 upper 32 (MEM64)>\n");
  } else {
    Print(L"  [RAW] BAR1: 0x%08x\n", I->B1.RawLo);
    if (EFI_ERROR(I->B1.St)) {
      Print(L"  [BAR] BAR1: probe failed: %r\n", I->B1.St);
    } else if (!I->B1.Size) {
      Print(L"  [BAR] BAR1: <unused or size=0>\n");
    } else {
      Print(L"  [BAR] BAR1: %s%s  Base=0x%016lx  Size=0x%016lx  End=0x%016lx\n", BarStr(I->B1.Type),
            I->B1.Pref ? L"(P)" : L"", I->B1.Base, I->B1.Size, I->B1.End);
    }
  }

  Print(L"\n  [RAW - Bridge Windows]\n");
  Print(L"    MEM_BASE(0x20)=0x%04x  MEM_LIMIT(0x22)=0x%04x\n", I->W.MemBaseRaw, I->W.MemLimitRaw);
  Print(L"    IO_BASE (0x1C)=0x%02x  IO_LIMIT (0x1D)=0x%02x\n", I->W.IoBaseRaw, I->W.IoLimitRaw);
  Print(L"    IO_BASE_UP16(0x30)=0x%04x  IO_LIMIT_UP16(0x32)=0x%04x\n", I->W.IoBaseUp, I->W.IoLimitUp);

  Print(L"  [DECODE - Bridge Windows]\n");
  if (!I->W.MemEn) {
    Print(L"  [WIN] Memory Window: <disabled> (Base=0x%08x Limit=0x%08x)\n", I->W.MemBase, I->W.MemLimit);
  } else {
    Print(L"  [WIN] Memory Window: Base=0x%08x  Limit=0x%08x  Size=0x%08x\n", I->W.MemBase, I->W.MemLimit,
          I->W.MemSize);
  }

  Print(L"  [WIN] IO Window type: %s\n", I->W.Io32 ? L"32-bit" : L"16-bit");
  if (!I->W.IoEn) {
    Print(L"  [WIN] IO Window: <disabled> (Base=0x%08x Limit=0x%08x)\n", I->W.IoBase, I->W.IoLimit);
  } else {
    Print(L"  [WIN] IO Window: Base=0x%08x  Limit=0x%08x  Size=0x%08x\n", I->W.IoBase, I->W.IoLimit, I->W.IoSize);
  }

  Print(L"\n  [RAW - Prefetchable Window]\n");
  Print(L"    PREF_MEM_BASE(0x24)=0x%04x  PREF_MEM_LIMIT(0x26)=0x%04x\n", I->W.PrefBaseRaw, I->W.PrefLimitRaw);
  Print(L"    PREF_BASE_UP32(0x28)=0x%08x  PREF_LIMIT_UP32(0x2C)=0x%08x\n", I->W.PrefBaseUp, I->W.PrefLimitUp);
}

/* ---------- Per-device ---------- */
static BOOLEAN PrintDev(UINTN Idx, UINTN Seg, UINTN Bus, UINTN Dev, UINTN Func, UINT8 *HdrTypeOut, UINT8 *BaseClassOut,
                        UINT8 *SubClassOut) {
  UINT16 Vid = 0;
  UINT16 Did = 0;
  UINT8 Cl = 0;
  UINT8 Sb = 0;
  UINT8 Pi = 0;
  UINT8 Hdr = 0;
  if (EFI_ERROR(R16(OFF_VID, &Vid))) {
    return FALSE;
  }
  if (Vid == 0xFFFF) {
    return FALSE;
  }

  R16(OFF_DID, &Did);
  R8(OFF_CLASS, &Cl);
  R8(OFF_SUB, &Sb);
  R8(OFF_PROGIF, &Pi);
  R8(OFF_HDR, &Hdr);
  UINT8 Ht = (UINT8)(Hdr & 0x7F);
  if (HdrTypeOut) {
    *HdrTypeOut = Ht;
  }
  if (BaseClassOut) {
    *BaseClassOut = Cl;
  }
  if (SubClassOut) {
    *SubClassOut = Sb;
  }

  Print(L"\n------------------------------------------------------------\n");
  Print(L"[DEV #%u] %04x:%02x:%02x.%x  VID:DID=%04x:%04x  Class=%02x%02x%02x  HdrType=0x%02x\n", Idx, Seg, Bus, Dev,
        Func, Vid, Did, Cl, Sb, Pi, Ht);

  UINT8 VpdOff = DumpStdCaps(Ht);
  DumpExtCaps();
  if (VpdOff) {
    DumpVpd(VpdOff);
  } else {
    Print(L"  [VPD] <not present>\n");
  }

  return TRUE;
}

static VOID DumpAllAndCollectBridges(VOID) {
  if (EFI_ERROR(InitMcfg())) {
    Print(L"InitMcfg failed, cannot scan devices.\n");
    return;
  }

  UINTN MaxEntries = McfgAllocCount;
  UINTN MaxBuses = 256;
  UINTN MaxDevs = 32;
  UINTN MaxFuncs = 8;
  UINTN MaxScan = MaxEntries * MaxBuses * MaxDevs * MaxFuncs;

  BRIDGE_INFO *B = AllocateZeroPool(sizeof(BRIDGE_INFO) * MaxScan);
  if (!B) {
    Print(L"AllocateZeroPool failed for bridges\n");
    return;
  }

  Print(L"\n============================================================\n");
  Print(L"[Step4-lspci] Dump Capabilities + VPD for each PCI/PCIe device (MCFG Entries=%u)\n", McfgAllocCount);

  UINTN DevCnt = 0;
  UINTN BrCnt = 0;

  for (UINTN i = 0; i < McfgAllocCount; i++) {
    UINTN Seg = McfgAllocs[i].PciSegmentGroupNumber;
    for (UINTN Bus = McfgAllocs[i].StartBusNumber; Bus <= McfgAllocs[i].EndBusNumber; Bus++) {
      for (UINTN Dev = 0; Dev < 32; Dev++) {
        UINT8 Ht = 0xFF;
        UINT8 Base = 0xFF;
        UINT8 Sub = 0xFF;
        BOOLEAN MultiFunc = FALSE;

        for (UINTN Func = 0; Func < 8; Func++) {
          if (Func > 0 && !MultiFunc) {
            break;
          }

          SetCurrentLocation(Seg, Bus, Dev, Func);
          if (Func == 0) {
            UINT16 Vid = 0;
            if (EFI_ERROR(R16(OFF_VID, &Vid)) || Vid == 0xFFFF) {
              break;
            }
            UINT8 Hdr = 0;
            if (!EFI_ERROR(R8(OFF_HDR, &Hdr))) {
              MultiFunc = ((Hdr & BIT7) != 0);
            }
          }

          if (!PrintDev(DevCnt + 1, Seg, Bus, Dev, Func, &Ht, &Base, &Sub)) {
            continue;
          }
          DevCnt++;

          // Use PCI Class Code to identify bridges (Base Class = 0x06)
          // Typical PCIe Root Ports / Switch Ports show as 0x0604 (PCI-to-PCI Bridge).
          // We filter to bridge-like subclasses that use Type-1 style bus numbers.
          if (Base == 0x06 && (Sub == 0x04 || Sub == 0x07)) {
            if (!EFI_ERROR(CollectBridge(Seg, Bus, Dev, Func, &B[BrCnt]))) {
              BrCnt++;
            }
          }
        }
      }
    }
  }

  Print(L"\n============================================================\n");
  Print(L"[Bridge Scan] Collect PCI-to-PCI Bridges into array and print...\n");
  Print(L"\nScan done. Total bridges found: %u\n", BrCnt);
  for (UINTN j = 0; j < BrCnt; j++) {
    PrintBridge(&B[j], j + 1);
  }

  FreePool(B);
}

/* ---------- EntryPoint ---------- */
EFI_STATUS EFIAPI ReadPciEntryPoint(IN EFI_HANDLE ImageHandle, IN EFI_SYSTEM_TABLE *SystemTable) {
  Print(L"[Step4-lspci] Dump ACPI MCFG + init ECAM cache...\n");
  DumpMcfg();

  Print(L"\n[Step4-lspci] Scan PCIe ECAM by MCFG (no PCI IO protocol)...\n");
  DumpAllAndCollectBridges();

  if (McfgAllocs) {
    FreePool(McfgAllocs);
    McfgAllocs = NULL;
    McfgAllocCount = 0;
    McfgReady = FALSE;
  }
  if (VpdBuf) {
    FreePool(VpdBuf);
    VpdBuf = NULL;
    VpdBufCap = 0;
  }
  return EFI_SUCCESS;
}
