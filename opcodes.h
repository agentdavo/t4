char* Primaries[] = {
        "J",   "LDLP",  "PFIX", "LDNL",
        "LDC", "LDNLP", "NFIX", "LDL",
        "ADC", "CALL",  "CJ",   "AJW",
        "EQC", "STL",   "STNL", "OPR"
};

char* Secondaries[] = {
/*00*/  "REV",          "LB",              "BSUB",         "ENDP",
/*04*/  "DIFF",         "ADD",             "GCALL",        "IN",
/*08*/  "PROD",         "GT",              "WSUB",         "OUT",
/*0C*/  "SUB",          "STARTP",          "OUTBYTE",      "OUTWORD",

/*10*/  "SETERR",       "--11--",          "RESETCH",      "CSUB0",
/*14*/  "--14--",       "STOPP",           "LADD",         "STLB",
/*18*/  "STHF",         "NORM",            "LDIV",         "LDPI",
/*1C*/  "STLF",         "XDBLE",           "LDPRI",        "REM",

/*20*/ "RET",           "LEND",            "LDTIMER",      "TESTLDS",
/*24*/ "TESTLDE",       "TESTLDD",         "TESTSTS",      "TESTSTE",
/*28*/ "TESTSTD",       "TESTERR",         "TESTPRANAL",   "TIN",
/*2C*/ "DIV",           "TESTHARDCHAN",    "DIST",         "DISC",

/*30*/ "DISS",          "LMUL",            "NOT",          "XOR",
/*34*/ "BCNT",          "LSHR",            "LSHL",         "LSUM",
/*38*/ "LSUB",          "RUNP",            "XWORD",        "SB",
/*3C*/ "GAJW",          "SAVEL",           "SAVEH",        "WCNT",

/*40*/ "SHR",           "SHL",             "MINT",         "ALT",
/*44*/ "ALTWT",         "ALTEND",          "AND",          "ENBT",
/*48*/ "ENBC",          "ENBS",            "MOVE",         "OR",
/*4C*/ "CSNGL",         "CCNT1",           "TALT",         "LDIFF",

/*50*/ "STHB",          "TALTWT",          "SUM",          "MUL",
/*54*/ "STTIMER",       "STOPERR",         "CWORD",        "CLRHALTERR",
/*58*/ "SETHALTERR",    "TESTHALTERR",     "DUP",          "MOVE2DINIT",
/*5C*/  "MOVE2DALL",    "MOVE2DNONZERO",   "MOVE2DZERO",   "--5F--",

/*60*/ "--60--",        "--61--",          "--62--",       "UNPACKSN",
/*64*/ "--64--",        "--65--",          "--66--",       "--67--",
/*68*/ "--68--",        "--69--",          "--6A--",       "--6B--",
/*6C*/ "POSTNORMSN",    "ROUNDSN",         "--6E--",       "--6F--",

/*70*/ "--70--",        "LDINF",           "FMUL",         "CFLERR",
/*74*/ "CRCWORD",       "CRCBYTE",         "BITCNT",       "BITREVWORD",
/*78*/ "BITREVNBITS",   "POP",         "--7A--",       "--7B--",
/*7C*/ "--7C--",        "--7D--",          "LDMEMSTARTVAL","--7F--",

/*80*/ "FPSTTEST",      "WSUBDB",          "FPLDNLDBI",    "FPCHKERR",
/*84*/ "FPSTNLDB",      "FPLDTEST",        "FPLDNLSNI",    "FPADD",
/*88*/ "FPSTNLSN",      "FPSUB",           "FPLDNLDB",     "FPMUL",
/*8C*/ "FPDIV",         "--8D--",          "FPLDNLSN",     "FPREMFIRST",

/*90*/ "FPREMSTEP",     "FPNAN",           "FPORDERED",    "FPNOTFINITE",
/*94*/ "FPGT",          "FPEQ",            "FPI32TOR32",   "--97--",
/*98*/ "FPI32TOR64",    "--99--",          "FPB32TOR64",   "--9B--",
/*9C*/ "FPTESTERR",     "FPRTOI32",        "FPSTNLI32",    "FPLDZEROSN",

/*A0*/ "FPLDZERODB",    "FPINT",           "--A2--",       "FPDUP",
/*A4*/ "FPREV",         "--A5--",          "FPLDNLADDDB",  "--A7--",
/*A8*/ "FPLDNLMULDB",   "--A9--",          "FPLDNLADDSN",  "FPENTRY",
/*AC*/ "FPLDNLMULSN",   "--AD--",          "--AE--",       "--AF--",

/*17C*/ "LDDEVID",
/*1FF*/ "START"
};
#define MAX_SECONDARIES 0xaf


/* IMS T9000 secondaries above the T800 range (ISM appendix B, table B.3) */
char* T9kOps[] = {
/*B0*/ "SETTIMESLICE","--B1--",        "--B2--",       "--B3--",
/*B4*/ "--B4--",      "--B5--",        "LDFLAGS",      "STFLAGS",
/*B8*/ "XBWORD",      "LBX",           "CB",           "CBU",
/*BC*/ "INSPHDR",     "READBFR",       "LDCONF",       "STCONF",
/*C0*/ "LDCNT",       "SSUB",          "LDTH",         "LDCHSTATUS",
/*C4*/ "INTDIS",      "INTENB",        "--C6--",       "CIR",
/*C8*/ "SS",          "CHANTYPE",      "LS",           "FPSETERR",
/*CC*/ "CIRU",        "--CD--",        "--CE--",       "FPREM",
/*D0*/ "FPRN",        "FPDIVBY2",      "FPMULBY2",     "FPSQRT",
/*D4*/ "FPRP",        "FPRM",          "FPRZ",         "FPR32TOR64",
/*D8*/ "FPR64TOR32",  "FPEXPDEC32",    "FPEXPINC32",   "FPABS",
/*DC*/ "FPCLRERR",    "FPADDDBSN",     "FPCHKI32",     "FPCHKI64",
/*E0*/ "--E0--",      "--E1--",        "--E2--",       "--E3--",
/*E4*/ "--E4--",      "--E5--",        "--E6--",       "--E7--",
/*E8*/ "--E8--",      "--E9--",        "--EA--",       "--EB--",
/*EC*/ "--EC--",      "--ED--",        "--EE--",       "--EF--",
/*F0*/ "DEVLB",       "DEVSB",         "DEVLS",        "DEVSS",
/*F4*/ "DEVLW",       "DEVSW",         "--F6--",       "--F7--",
/*F8*/ "XSWORD",      "LSX",           "CS",           "CSU"
};
#define MIN_T9KOPS 0xb0
#define MAX_T9KOPS 0xfb

/* IMS T9000 negative-prefix secondaries, indexed by -OReg (table B.4) */
char* T9kNegOps[] = {
/*-00*/ "--N00--",    "FPSTALL",       "FPLDALL",      "STSHADOW",
/*-04*/ "LDSHADOW",   "TRET",          "GOPROT",       "SELTH",
/*-08*/ "SYSCALL",    "--N09--",       "--N0A--",      "WAIT",
/*-0C*/ "SIGNAL",     "TIMESLICE",     "INSERTQUEUE",  "SWAPTIMER",
/*-10*/ "SWAPQUEUE",  "--N11--",       "STOPCH",       "VOUT",
/*-14*/ "VIN",        "--N15--",       "--N16--",      "SWAPBFR",
/*-18*/ "SETHDR",     "SETCHMODE",     "INITVLCB",     "WRITEHDR",
/*-1C*/ "READHDR",    "DISG",          "ENBG",         "GRANT",
/*-20*/ "STMOVE2DINIT","CAUSEERROR",   "RESTART",      "UNMKRC",
/*-24*/ "MKRC",       "IRDSQ",         "ERDSQ",        "STRESPTR",
/*-28*/ "LDRESPTR",   "--N29--",       "--N2A--",      "--N2B--",
/*-2C*/ "DEVMOVE",    "ICL",           "FDCL",         "ICA",
/*-30*/ "FDCA"
};
#define MAX_T9KNEGOPS 0x30


char* FpuEntries[] = {
/*00*/ "--FPU00--",   "FPUSQRTFIRST", "FPUSQRTSTEP", "FPUSQRTLAST",
/*04*/ "FPURP",       "FPURM",        "FPURZ",       "FPUR32TOR64",
/*08*/ "FPUR64TOR32", "FPUEXPDEC32",  "FPUEXPINC32", "FPUABS",
/*0C*/ "--FPU0C--",   "FPUNOROUND",   "FPUCHKI32",   "FPUCHKI64",
/*10*/ "--FPU10--",   "FPUDIVBY2",    "FPUMULBY2",   "--FPU13--",

/*22*/ "FPURN",
/*23*/ "FPUSETERR",
/*9C*/ "FPUCLRERR"
};
#define MAX_FPUENTRIES 0x13


char* Combinations[] = {
/*100*/ "STL+LDL",       "LDL+LDNL",      "EQC+CJ",     "LDL+LDNLP",
/*104*/ "LDL+LDL",       "LDLP+FPLDNLDB", "AJW+RET",    "LDLP+LDC",
/*108*/ "LDLP+LDL",      "STL+STL",       "LDL+CFLERR", "LDLP+FPSTNLSN",
/*10C*/ "LDLP+FPLDNLSN", "LDL+ADC",       "LDL+STNL",   "LDC+LDL",
/*110*/ "STL+J",         "GT+CJ",         "WSUB+STNL",  "LDL+WSUB",
/*114*/ "DUP+STL",       "MINT+AND",      "--C16--",    "--C17--"
};
#define MIN_COMBINATIONS 0x100
#define MAX_COMBINATIONS 0x117


/* TVX - Transputer Vector eXtension (MXFP4 / bfloat16), OPR 0x1C0..0x1E1 */
char* TvxOps[] = {
/*1C0*/ "BF16TOR32",   "R32TOBF16",  "BF16PACK",   "BF16UNPKH",
/*1C4*/ "MXFP4TOR32",  "R32TOMXFP4", "MXE8M0TOR32","MXE8M0OF",
/*1C8*/ "MXFP4DOT",    "BF16DOT",    "MXFP4GEMV",  "MXFP4UNPK8",
/*1CC*/ "R32TOMXFP4X8","BF16MUL2",   "BF16ADD2",   "VSETLEN",
/*1D0*/ "MXFP4GEMVBF", "VADDSN",     "VMULSN",     "VSILUSN",
/*1D4*/ "VRMSNORM",    "VSOFTMAXSN", "F16TOR32",   "R32TOF16",
/*1D8*/ "Q8DOT",       "Q8DOTBF",    "IQ1MDOT",    "IQ2XXSDOT",
/*1DC*/ "IQ3XXSDOT",   "QSETFMT",    "QDOT",       "VHAD32",
/*1E0*/ "MXQROT",      "MXQROTBF",   "FTSETVP",    "FTRANSP",
/*1E4*/ "FTRANSPB",    "FSINCOS",    "FRSQRT",     "FRECIP",
/*1E8*/ "FSINCOSB",    "VSETSTRIDE", "FTCULL",     "TSKIN",
/*1EC*/ "VSETFMT",     "FTSETLIT",   "FLITB",      "QGEMM",
/*1F0*/ "VBLEND8",     "VFILL",      "VFILL2D",    "CSUM",
/*1F4*/ "VBLEND2D",    "STVXSTATE",  "CSUMCPY",    "VQ8K",
/*1F8*/ "VQ80",        "VCVT"
};
#define MIN_TVX 0x1c0
#define MAX_TVX 0x1f9
