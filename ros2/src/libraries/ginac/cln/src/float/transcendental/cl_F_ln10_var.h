{
#if CL_DS_BIG_ENDIAN_P
  #if (intDsize==8)
     D1(0x93), D1(0x5D), D1(0x8D), D1(0xDD), D1(0xAA), D1(0xA8), D1(0xAC), D1(0x17)
  #endif
  #if (intDsize==16)
     D2(0x93,0x5D), D2(0x8D,0xDD), D2(0xAA,0xA8), D2(0xAC,0x17)
  #endif
  #if (intDsize==32)
     D4(0x93,0x5D,0x8D,0xDD), D4(0xAA,0xA8,0xAC,0x17)
  #endif
  #if (intDsize==64)
     D8(0x93,0x5D,0x8D,0xDD,0xAA,0xA8,0xAC,0x17)
  #endif
#else
  #if (intDsize==8)
    D1(0x17), D1(0xAC), D1(0xA8), D1(0xAA), D1(0xDD), D1(0x8D), D1(0x5D), D1(0x93)
  #endif
  #if (intDsize==16)
    D2(0xAC,0x17), D2(0xAA,0xA8), D2(0x8D,0xDD), D2(0x93,0x5D)
  #endif
  #if (intDsize==32)
    D4(0xAA,0xA8,0xAC,0x17), D4(0x93,0x5D,0x8D,0xDD)
  #endif
  #if (intDsize==64)
    D8(0x93,0x5D,0x8D,0xDD,0xAA,0xA8,0xAC,0x17)
  #endif
#endif
} ;
