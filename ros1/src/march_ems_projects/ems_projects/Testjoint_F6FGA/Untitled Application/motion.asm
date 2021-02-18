;**** Assembler File: C:\Program Files (x86)\Technosoft\ESM\Projects\March 5 current tuning\Testjoint_F6FGA\Untitled Application\motion.asm
;****		generated with Technosoft Motion Language Compiler
;**************************************************
	.sect "MLP"
	.global _PROG_LOAD_ADDR
_PROG_LOAD_ADDR	.set	 04000h
	.global _PROG_RUN_ADDR
_PROG_RUN_ADDR :
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MOTION.TML
;*** 1	----------
;*** 2	----------
;*** 3	----------
;***		----------	BEGIN
	.global _PROG_INI
_PROG_INI :
	.word 0649Ch
;*** 4	----------
;*** 5	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\COPY_SEQUENCE.TML
;*** 1	----------
;*** 2	----------
;*** 3	----------
;*** 4	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MOTION.TML
;*** 6	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\SETUP_INTERRUPTS.TML
;*** 1	----------
;*** 2	----------
;*** 3	----------
;*** 4	----------
;***		----------	EXECUTEMAIN = MAINSTARTADDRESS
	.word 023BFh
	.word MAINSTARTADDRESS
;*** 5	----------
;***		----------	EXECUTEAUTOTUNING = CONTINUEWITHENDINIT
	.word 02398h
	.word CONTINUEWITHENDINIT
;*** 6	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MOTION.TML
;*** 7	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\INIT_CAM.TML
;*** 1	----------
;*** 2	----------
;*** 3	----------
;*** 4	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MOTION.TML
;*** 8	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\SETUP_SETTINGS.TML
;*** 1	----------
;*** 2	----------
;*** 3	----------
;*** 4	----------
;***		----------	GOTO CONTINUEWITHENDINIT, DOTMLINITIALIZATIONS, EQ
	.word 074C0h
	.word 009C0h
	.word CONTINUEWITHENDINIT
;*** 5	----------
;*** 6	----------
;***		----------	RET
	.word 00404h
;*** 7	----------
;*** 8	----------
;***		----------	CONTINUEWITHENDINIT :
	.global CONTINUEWITHENDINIT
CONTINUEWITHENDINIT :
;*** 9	----------
;*** 10	----------
;***		----------	SRB UPGRADE, 0xFFFF, 0x8000
	.word 05A57h
	.word 0FFFFh
	.word 08000h
;*** 11	----------
;*** 12	----------
;***		----------	ENDINIT
	.word 00020h
;*** 13	----------
;*** 14	----------
;***		----------	WAIT_VDC :
	.global WAIT_VDC
WAIT_VDC :
;*** 15	----------
;*** 16	----------
;***		----------	GOTO WAIT_VDC, FLAGUV, GT
	.word 07484h
	.word 002FBh
	.word WAIT_VDC
;*** 17	----------
;*** 18	----------
;***		----------	AXISON
	.word 00102h
;*** 19	----------
;*** 20	----------
;***		----------	MAINSTARTADDRESS :
	.global MAINSTARTADDRESS
MAINSTARTADDRESS :
;*** 21	----------
;*** 22	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MOTION.TML
;*** 9	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MAIN.TML
;*** 1	----------
;*** 2	----------
;*** 3	----------
;*** 4	----------
;*** 5	----------
;***		----------	CACC = 0.02086(0x00000557)
	.word 024A2h
	.word 00557h
	.word 00000h
;*** 6	----------
;***		----------	CSPD = 6.55360(0x00068DB9)
	.word 024A0h
	.word 08DB9h
	.word 00006h
;*** 7	----------
;***		----------	CPOS = 115343(0x0001C28F)
	.word 0249Eh
	.word 0C28Fh
	.word 00001h
;*** 8	----------
;***		----------	CPA
	.word 05909h
	.word 0FFFFh
	.word 02000h
;*** 9	----------
;***		----------	MODE PP
	.word 05909h
	.word 0BFC1h
	.word 08701h
;*** 10	----------
;***		----------	TUM1
	.word 05909h
	.word 0FFFFh
	.word 04000h
;*** 11	----------
;***		----------	UPD
	.word 00108h
;*** 12	----------
;***		----------	!MC
	.word 0700Fh
;***		----------	WAIT!
	.word 00408h
;*** 13	----------
;*** 14	----------
;*** 15	----------
;***		----------	CACC = 0.02086(0x00000557)
	.word 024A2h
	.word 00557h
	.word 00000h
;*** 16	----------
;***		----------	CSPD = 2.18454(0x00022F3E)
	.word 024A0h
	.word 02F3Eh
	.word 00002h
;*** 17	----------
;***		----------	CPOS = 98304(0x00018000)
	.word 0249Eh
	.word 08000h
	.word 00001h
;*** 18	----------
;***		----------	CPA
	.word 05909h
	.word 0FFFFh
	.word 02000h
;*** 19	----------
;***		----------	MODE PP
	.word 05909h
	.word 0BFC1h
	.word 08701h
;*** 20	----------
;***		----------	TUM1
	.word 05909h
	.word 0FFFFh
	.word 04000h
;*** 21	----------
;***		----------	UPD
	.word 00108h
;*** 22	----------
;***		----------	!MC
	.word 0700Fh
;***		----------	WAIT!
	.word 00408h
;*** 23	----------
;*** 24	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MOTION.TML
;*** 10	----------
;*** 11	----------
;***		----------	END
	.word 00001h
;*** 12	----------
;*** 13	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\HOMINGMODES.TML
;*** 1	----------
;*** 2	----------
;*** 3	----------
;*** 4	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MOTION.TML
;*** 14	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\FUNCTIONS.TML
;*** 1	----------
;*** 2	----------
;*** 3	----------
;*** 4	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MOTION.TML
;*** 15	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\INTERRUPTS.TML
;*** 1	----------
;*** 2	----------
;*** 3	----------
;*** 4	----------
;***		----------	File : C:\PROGRAM FILES (X86)\TECHNOSOFT\ESM\PROJECTS\MARCH 5 CURRENT TUNING\TESTJOINT_F6FGA\UNTITLED APPLICATION\MOTION.TML
;*** 16	----------
;*** 17	----------
	.global _PROG_END_ADDR
_PROG_END_ADDR :
	.word 00000h
