{
#if CL_DS_BIG_ENDIAN_P
  #if (intDsize==8)
     D1(0xAD), D1(0xF8), D1(0x54), D1(0x58), D1(0xA2), D1(0xBB), D1(0x4A), D1(0x9B)
  #endif
  #if (intDsize==16)
     D2(0xAD,0xF8), D2(0x54,0x58), D2(0xA2,0xBB), D2(0x4A,0x9B)
  #endif
  #if (intDsize==32)
     D4(0xAD,0xF8,0x54,0x58), D4(0xA2,0xBB,0x4A,0x9B)
  #endif
  #if (intDsize==64)
     D8(0xAD,0xF8,0x54,0x58,0xA2,0xBB,0x4A,0x9B)
  #endif
#else
  #if (intDsize==8)
    D1(0x9B), D1(0x4A), D1(0xBB), D1(0xA2), D1(0x58), D1(0x54), D1(0xF8), D1(0xAD)
  #endif
  #if (intDsize==16)
    D2(0x4A,0x9B), D2(0xA2,0xBB), D2(0x54,0x58), D2(0xAD,0xF8)
  #endif
  #if (intDsize==32)
    D4(0xA2,0xBB,0x4A,0x9B), D4(0xAD,0xF8,0x54,0x58)
  #endif
  #if (intDsize==64)
    D8(0xAD,0xF8,0x54,0x58,0xA2,0xBB,0x4A,0x9B)
  #endif
#endif
} ;
