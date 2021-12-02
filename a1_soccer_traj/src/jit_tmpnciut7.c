/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) jit_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s1[7] = {4, 4, 0, 0, 0, 0, 0};
static const casadi_int casadi_s2[73] = {9, 7, 0, 9, 18, 27, 36, 45, 54, 63, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s3[143] = {4, 28, 0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};

/* fwd7_T_fk:(i0[9],out_o0[4x4,0nz],fwd_i0[9x7])->(fwd_o0[4x28]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a8, a9;
  a0=arg[0]? arg[0][8] : 0;
  a1=cos(a0);
  a2=arg[0]? arg[0][7] : 0;
  a3=cos(a2);
  a4=arg[0]? arg[0][5] : 0;
  a5=cos(a4);
  a6=arg[0]? arg[0][4] : 0;
  a7=sin(a6);
  a8=arg[2]? arg[2][4] : 0;
  a9=(a7*a8);
  a10=(a5*a9);
  a11=cos(a6);
  a12=sin(a4);
  a13=arg[2]? arg[2][5] : 0;
  a14=(a12*a13);
  a15=(a11*a14);
  a10=(a10+a15);
  a15=(a3*a10);
  a16=(a11*a5);
  a17=sin(a2);
  a18=arg[2]? arg[2][7] : 0;
  a19=(a17*a18);
  a20=(a16*a19);
  a15=(a15+a20);
  a20=sin(a2);
  a21=arg[0]? arg[0][6] : 0;
  a22=sin(a21);
  a23=cos(a4);
  a13=(a23*a13);
  a24=(a11*a13);
  a4=sin(a4);
  a25=(a4*a9);
  a24=(a24-a25);
  a25=(a22*a24);
  a26=(a11*a4);
  a27=cos(a21);
  a28=arg[2]? arg[2][6] : 0;
  a29=(a27*a28);
  a30=(a26*a29);
  a25=(a25+a30);
  a30=cos(a21);
  a31=cos(a6);
  a8=(a31*a8);
  a32=(a30*a8);
  a6=sin(a6);
  a21=sin(a21);
  a28=(a21*a28);
  a33=(a6*a28);
  a32=(a32-a33);
  a25=(a25+a32);
  a32=(a20*a25);
  a33=(a26*a22);
  a34=(a6*a30);
  a33=(a33+a34);
  a2=cos(a2);
  a18=(a2*a18);
  a34=(a33*a18);
  a32=(a32+a34);
  a15=(a15+a32);
  a32=(a1*a15);
  a34=(a16*a3);
  a35=(a33*a20);
  a34=(a34-a35);
  a35=sin(a0);
  a36=arg[2]? arg[2][8] : 0;
  a37=(a35*a36);
  a38=(a34*a37);
  a32=(a32+a38);
  a38=sin(a0);
  a39=(a16*a18);
  a40=(a20*a10);
  a39=(a39-a40);
  a25=(a3*a25);
  a40=(a33*a19);
  a25=(a25-a40);
  a39=(a39+a25);
  a25=(a38*a39);
  a40=(a16*a20);
  a41=(a33*a3);
  a40=(a40+a41);
  a0=cos(a0);
  a36=(a0*a36);
  a41=(a40*a36);
  a25=(a25+a41);
  a32=(a32+a25);
  a32=(-a32);
  if (res[0]!=0) res[0][0]=a32;
  a32=arg[0]? arg[0][3] : 0;
  a25=cos(a32);
  a41=arg[2]? arg[2][3] : 0;
  a42=(a25*a41);
  a43=(a6*a42);
  a44=sin(a32);
  a45=(a44*a8);
  a43=(a43+a45);
  a45=(a5*a43);
  a46=(a44*a6);
  a47=(a46*a14);
  a45=(a45-a47);
  a47=cos(a32);
  a48=(a47*a13);
  a32=sin(a32);
  a41=(a32*a41);
  a49=(a4*a41);
  a48=(a48-a49);
  a45=(a45+a48);
  a48=(a3*a45);
  a49=(a46*a5);
  a50=(a47*a4);
  a49=(a49+a50);
  a50=(a49*a19);
  a48=(a48-a50);
  a50=(a47*a5);
  a51=(a46*a4);
  a50=(a50-a51);
  a51=(a50*a29);
  a52=(a5*a41);
  a53=(a47*a14);
  a52=(a52+a53);
  a43=(a4*a43);
  a53=(a46*a13);
  a43=(a43+a53);
  a52=(a52+a43);
  a43=(a22*a52);
  a51=(a51-a43);
  a43=(a11*a42);
  a53=(a44*a9);
  a43=(a43-a53);
  a53=(a30*a43);
  a54=(a44*a11);
  a55=(a54*a28);
  a53=(a53-a55);
  a51=(a51+a53);
  a53=(a20*a51);
  a55=(a50*a22);
  a56=(a54*a30);
  a55=(a55+a56);
  a56=(a55*a18);
  a53=(a53+a56);
  a48=(a48+a53);
  a53=(a1*a48);
  a56=(a49*a3);
  a57=(a55*a20);
  a56=(a56+a57);
  a57=(a56*a37);
  a53=(a53-a57);
  a57=(a20*a45);
  a58=(a49*a18);
  a57=(a57+a58);
  a51=(a3*a51);
  a58=(a55*a19);
  a51=(a51-a58);
  a57=(a57-a51);
  a51=(a38*a57);
  a58=(a49*a20);
  a59=(a55*a3);
  a58=(a58-a59);
  a59=(a58*a36);
  a51=(a51+a59);
  a53=(a53-a51);
  if (res[0]!=0) res[0][1]=a53;
  a53=(a4*a42);
  a51=(a44*a13);
  a53=(a53+a51);
  a51=(a47*a8);
  a59=(a6*a41);
  a51=(a51-a59);
  a59=(a5*a51);
  a60=(a47*a6);
  a61=(a60*a14);
  a59=(a59-a61);
  a53=(a53-a59);
  a59=(a3*a53);
  a61=(a44*a4);
  a62=(a60*a5);
  a61=(a61-a62);
  a62=(a61*a19);
  a59=(a59-a62);
  a62=(a47*a11);
  a63=(a62*a30);
  a64=(a60*a4);
  a65=(a44*a5);
  a64=(a64+a65);
  a65=(a64*a22);
  a63=(a63-a65);
  a65=(a63*a18);
  a41=(a11*a41);
  a9=(a47*a9);
  a41=(a41+a9);
  a9=(a30*a41);
  a66=(a62*a28);
  a9=(a9+a66);
  a51=(a4*a51);
  a13=(a60*a13);
  a51=(a51+a13);
  a42=(a5*a42);
  a14=(a44*a14);
  a42=(a42-a14);
  a51=(a51+a42);
  a42=(a22*a51);
  a14=(a64*a29);
  a42=(a42+a14);
  a9=(a9+a42);
  a42=(a20*a9);
  a65=(a65-a42);
  a59=(a59-a65);
  a65=(a1*a59);
  a42=(a61*a3);
  a14=(a63*a20);
  a42=(a42-a14);
  a14=(a42*a37);
  a65=(a65-a14);
  a14=(a20*a53);
  a18=(a61*a18);
  a14=(a14+a18);
  a9=(a3*a9);
  a19=(a63*a19);
  a9=(a9+a19);
  a14=(a14-a9);
  a9=(a38*a14);
  a19=(a61*a20);
  a18=(a63*a3);
  a19=(a19+a18);
  a18=(a19*a36);
  a9=(a9+a18);
  a65=(a65-a9);
  if (res[0]!=0) res[0][2]=a65;
  a65=0.;
  if (res[0]!=0) res[0][3]=a65;
  a8=(a22*a8);
  a9=(a6*a29);
  a8=(a8+a9);
  a9=(a30*a24);
  a18=(a26*a28);
  a9=(a9-a18);
  a8=(a8-a9);
  if (res[0]!=0) res[0][4]=a8;
  a9=(a30*a52);
  a18=(a50*a28);
  a9=(a9+a18);
  a43=(a22*a43);
  a18=(a54*a29);
  a43=(a43+a18);
  a9=(a9+a43);
  a43=(-a9);
  if (res[0]!=0) res[0][5]=a43;
  a43=(a30*a51);
  a28=(a64*a28);
  a43=(a43-a28);
  a29=(a62*a29);
  a41=(a22*a41);
  a29=(a29-a41);
  a43=(a43+a29);
  if (res[0]!=0) res[0][6]=a43;
  if (res[0]!=0) res[0][7]=a65;
  a29=(a34*a36);
  a15=(a38*a15);
  a29=(a29-a15);
  a15=(a1*a39);
  a41=(a40*a37);
  a15=(a15-a41);
  a29=(a29+a15);
  if (res[0]!=0) res[0][8]=a29;
  a48=(a38*a48);
  a15=(a56*a36);
  a48=(a48+a15);
  a15=(a1*a57);
  a41=(a58*a37);
  a15=(a15-a41);
  a48=(a48+a15);
  if (res[0]!=0) res[0][9]=a48;
  a59=(a38*a59);
  a36=(a42*a36);
  a59=(a59+a36);
  a36=(a1*a14);
  a37=(a19*a37);
  a36=(a36-a37);
  a59=(a59+a36);
  if (res[0]!=0) res[0][10]=a59;
  if (res[0]!=0) res[0][11]=a65;
  a36=-2.0000000000000001e-001;
  a29=(a36*a29);
  a39=(a36*a39);
  a37=-8.5050000000000001e-002;
  a8=(a37*a8);
  a15=arg[2]? arg[2][0] : 0;
  a41=1.8300000000000000e-001;
  a10=(a41*a10);
  a28=-4.7000000000000000e-002;
  a24=(a28*a24);
  a10=(a10+a24);
  a15=(a15-a10);
  a8=(a8+a15);
  a39=(a39+a8);
  a29=(a29+a39);
  if (res[0]!=0) res[0][12]=a29;
  a48=(a36*a48);
  a57=(a36*a57);
  a45=(a41*a45);
  a52=(a28*a52);
  a45=(a45-a52);
  a52=arg[2]? arg[2][1] : 0;
  a45=(a45+a52);
  a9=(a37*a9);
  a45=(a45-a9);
  a57=(a57+a45);
  a48=(a48+a57);
  if (res[0]!=0) res[0][13]=a48;
  a59=(a36*a59);
  a14=(a36*a14);
  a43=(a37*a43);
  a53=(a41*a53);
  a51=(a28*a51);
  a53=(a53+a51);
  a51=arg[2]? arg[2][2] : 0;
  a53=(a53+a51);
  a43=(a43+a53);
  a14=(a14+a43);
  a59=(a59+a14);
  if (res[0]!=0) res[0][14]=a59;
  if (res[0]!=0) res[0][15]=a65;
  a59=arg[2]? arg[2][13] : 0;
  a14=(a7*a59);
  a43=(a5*a14);
  a53=arg[2]? arg[2][14] : 0;
  a51=(a12*a53);
  a48=(a11*a51);
  a43=(a43+a48);
  a48=(a3*a43);
  a57=arg[2]? arg[2][16] : 0;
  a45=(a17*a57);
  a9=(a16*a45);
  a48=(a48+a9);
  a53=(a23*a53);
  a9=(a11*a53);
  a52=(a4*a14);
  a9=(a9-a52);
  a52=(a22*a9);
  a29=arg[2]? arg[2][15] : 0;
  a39=(a27*a29);
  a8=(a26*a39);
  a52=(a52+a8);
  a59=(a31*a59);
  a8=(a30*a59);
  a29=(a21*a29);
  a15=(a6*a29);
  a8=(a8-a15);
  a52=(a52+a8);
  a8=(a20*a52);
  a57=(a2*a57);
  a15=(a33*a57);
  a8=(a8+a15);
  a48=(a48+a8);
  a8=(a1*a48);
  a15=arg[2]? arg[2][17] : 0;
  a10=(a35*a15);
  a24=(a34*a10);
  a8=(a8+a24);
  a24=(a16*a57);
  a18=(a20*a43);
  a24=(a24-a18);
  a52=(a3*a52);
  a18=(a33*a45);
  a52=(a52-a18);
  a24=(a24+a52);
  a52=(a38*a24);
  a15=(a0*a15);
  a18=(a40*a15);
  a52=(a52+a18);
  a8=(a8+a52);
  a8=(-a8);
  if (res[0]!=0) res[0][16]=a8;
  a8=arg[2]? arg[2][12] : 0;
  a52=(a25*a8);
  a18=(a6*a52);
  a13=(a44*a59);
  a18=(a18+a13);
  a13=(a5*a18);
  a66=(a46*a51);
  a13=(a13-a66);
  a66=(a47*a53);
  a8=(a32*a8);
  a67=(a4*a8);
  a66=(a66-a67);
  a13=(a13+a66);
  a66=(a3*a13);
  a67=(a49*a45);
  a66=(a66-a67);
  a67=(a50*a39);
  a68=(a5*a8);
  a69=(a47*a51);
  a68=(a68+a69);
  a18=(a4*a18);
  a69=(a46*a53);
  a18=(a18+a69);
  a68=(a68+a18);
  a18=(a22*a68);
  a67=(a67-a18);
  a18=(a11*a52);
  a69=(a44*a14);
  a18=(a18-a69);
  a69=(a30*a18);
  a70=(a54*a29);
  a69=(a69-a70);
  a67=(a67+a69);
  a69=(a20*a67);
  a70=(a55*a57);
  a69=(a69+a70);
  a66=(a66+a69);
  a69=(a1*a66);
  a70=(a56*a10);
  a69=(a69-a70);
  a70=(a20*a13);
  a71=(a49*a57);
  a70=(a70+a71);
  a67=(a3*a67);
  a71=(a55*a45);
  a67=(a67-a71);
  a70=(a70-a67);
  a67=(a38*a70);
  a71=(a58*a15);
  a67=(a67+a71);
  a69=(a69-a67);
  if (res[0]!=0) res[0][17]=a69;
  a69=(a4*a52);
  a67=(a44*a53);
  a69=(a69+a67);
  a67=(a47*a59);
  a71=(a6*a8);
  a67=(a67-a71);
  a71=(a5*a67);
  a72=(a60*a51);
  a71=(a71-a72);
  a69=(a69-a71);
  a71=(a3*a69);
  a72=(a61*a45);
  a71=(a71-a72);
  a72=(a63*a57);
  a8=(a11*a8);
  a14=(a47*a14);
  a8=(a8+a14);
  a14=(a30*a8);
  a73=(a62*a29);
  a14=(a14+a73);
  a67=(a4*a67);
  a53=(a60*a53);
  a67=(a67+a53);
  a52=(a5*a52);
  a51=(a44*a51);
  a52=(a52-a51);
  a67=(a67+a52);
  a52=(a22*a67);
  a51=(a64*a39);
  a52=(a52+a51);
  a14=(a14+a52);
  a52=(a20*a14);
  a72=(a72-a52);
  a71=(a71-a72);
  a72=(a1*a71);
  a52=(a42*a10);
  a72=(a72-a52);
  a52=(a20*a69);
  a57=(a61*a57);
  a52=(a52+a57);
  a14=(a3*a14);
  a45=(a63*a45);
  a14=(a14+a45);
  a52=(a52-a14);
  a14=(a38*a52);
  a45=(a19*a15);
  a14=(a14+a45);
  a72=(a72-a14);
  if (res[0]!=0) res[0][18]=a72;
  if (res[0]!=0) res[0][19]=a65;
  a59=(a22*a59);
  a72=(a6*a39);
  a59=(a59+a72);
  a72=(a30*a9);
  a14=(a26*a29);
  a72=(a72-a14);
  a59=(a59-a72);
  if (res[0]!=0) res[0][20]=a59;
  a72=(a30*a68);
  a14=(a50*a29);
  a72=(a72+a14);
  a18=(a22*a18);
  a14=(a54*a39);
  a18=(a18+a14);
  a72=(a72+a18);
  a18=(-a72);
  if (res[0]!=0) res[0][21]=a18;
  a18=(a30*a67);
  a29=(a64*a29);
  a18=(a18-a29);
  a39=(a62*a39);
  a8=(a22*a8);
  a39=(a39-a8);
  a18=(a18+a39);
  if (res[0]!=0) res[0][22]=a18;
  if (res[0]!=0) res[0][23]=a65;
  a39=(a34*a15);
  a48=(a38*a48);
  a39=(a39-a48);
  a48=(a1*a24);
  a8=(a40*a10);
  a48=(a48-a8);
  a39=(a39+a48);
  if (res[0]!=0) res[0][24]=a39;
  a66=(a38*a66);
  a48=(a56*a15);
  a66=(a66+a48);
  a48=(a1*a70);
  a8=(a58*a10);
  a48=(a48-a8);
  a66=(a66+a48);
  if (res[0]!=0) res[0][25]=a66;
  a71=(a38*a71);
  a15=(a42*a15);
  a71=(a71+a15);
  a15=(a1*a52);
  a10=(a19*a10);
  a15=(a15-a10);
  a71=(a71+a15);
  if (res[0]!=0) res[0][26]=a71;
  if (res[0]!=0) res[0][27]=a65;
  a39=(a36*a39);
  a24=(a36*a24);
  a59=(a37*a59);
  a15=arg[2]? arg[2][9] : 0;
  a43=(a41*a43);
  a9=(a28*a9);
  a43=(a43+a9);
  a15=(a15-a43);
  a59=(a59+a15);
  a24=(a24+a59);
  a39=(a39+a24);
  if (res[0]!=0) res[0][28]=a39;
  a66=(a36*a66);
  a70=(a36*a70);
  a13=(a41*a13);
  a68=(a28*a68);
  a13=(a13-a68);
  a68=arg[2]? arg[2][10] : 0;
  a13=(a13+a68);
  a72=(a37*a72);
  a13=(a13-a72);
  a70=(a70+a13);
  a66=(a66+a70);
  if (res[0]!=0) res[0][29]=a66;
  a71=(a36*a71);
  a52=(a36*a52);
  a18=(a37*a18);
  a69=(a41*a69);
  a67=(a28*a67);
  a69=(a69+a67);
  a67=arg[2]? arg[2][11] : 0;
  a69=(a69+a67);
  a18=(a18+a69);
  a52=(a52+a18);
  a71=(a71+a52);
  if (res[0]!=0) res[0][30]=a71;
  if (res[0]!=0) res[0][31]=a65;
  a71=arg[2]? arg[2][22] : 0;
  a52=(a7*a71);
  a18=(a5*a52);
  a69=arg[2]? arg[2][23] : 0;
  a67=(a12*a69);
  a66=(a11*a67);
  a18=(a18+a66);
  a66=(a3*a18);
  a70=arg[2]? arg[2][25] : 0;
  a13=(a17*a70);
  a72=(a16*a13);
  a66=(a66+a72);
  a69=(a23*a69);
  a72=(a11*a69);
  a68=(a4*a52);
  a72=(a72-a68);
  a68=(a22*a72);
  a39=arg[2]? arg[2][24] : 0;
  a24=(a27*a39);
  a59=(a26*a24);
  a68=(a68+a59);
  a71=(a31*a71);
  a59=(a30*a71);
  a39=(a21*a39);
  a15=(a6*a39);
  a59=(a59-a15);
  a68=(a68+a59);
  a59=(a20*a68);
  a70=(a2*a70);
  a15=(a33*a70);
  a59=(a59+a15);
  a66=(a66+a59);
  a59=(a1*a66);
  a15=arg[2]? arg[2][26] : 0;
  a43=(a35*a15);
  a9=(a34*a43);
  a59=(a59+a9);
  a9=(a16*a70);
  a10=(a20*a18);
  a9=(a9-a10);
  a68=(a3*a68);
  a10=(a33*a13);
  a68=(a68-a10);
  a9=(a9+a68);
  a68=(a38*a9);
  a15=(a0*a15);
  a10=(a40*a15);
  a68=(a68+a10);
  a59=(a59+a68);
  a59=(-a59);
  if (res[0]!=0) res[0][32]=a59;
  a59=arg[2]? arg[2][21] : 0;
  a68=(a25*a59);
  a10=(a6*a68);
  a48=(a44*a71);
  a10=(a10+a48);
  a48=(a5*a10);
  a8=(a46*a67);
  a48=(a48-a8);
  a8=(a47*a69);
  a59=(a32*a59);
  a29=(a4*a59);
  a8=(a8-a29);
  a48=(a48+a8);
  a8=(a3*a48);
  a29=(a49*a13);
  a8=(a8-a29);
  a29=(a50*a24);
  a14=(a5*a59);
  a45=(a47*a67);
  a14=(a14+a45);
  a10=(a4*a10);
  a45=(a46*a69);
  a10=(a10+a45);
  a14=(a14+a10);
  a10=(a22*a14);
  a29=(a29-a10);
  a10=(a11*a68);
  a45=(a44*a52);
  a10=(a10-a45);
  a45=(a30*a10);
  a57=(a54*a39);
  a45=(a45-a57);
  a29=(a29+a45);
  a45=(a20*a29);
  a57=(a55*a70);
  a45=(a45+a57);
  a8=(a8+a45);
  a45=(a1*a8);
  a57=(a56*a43);
  a45=(a45-a57);
  a57=(a20*a48);
  a51=(a49*a70);
  a57=(a57+a51);
  a29=(a3*a29);
  a51=(a55*a13);
  a29=(a29-a51);
  a57=(a57-a29);
  a29=(a38*a57);
  a51=(a58*a15);
  a29=(a29+a51);
  a45=(a45-a29);
  if (res[0]!=0) res[0][33]=a45;
  a45=(a4*a68);
  a29=(a44*a69);
  a45=(a45+a29);
  a29=(a47*a71);
  a51=(a6*a59);
  a29=(a29-a51);
  a51=(a5*a29);
  a53=(a60*a67);
  a51=(a51-a53);
  a45=(a45-a51);
  a51=(a3*a45);
  a53=(a61*a13);
  a51=(a51-a53);
  a53=(a63*a70);
  a59=(a11*a59);
  a52=(a47*a52);
  a59=(a59+a52);
  a52=(a30*a59);
  a73=(a62*a39);
  a52=(a52+a73);
  a29=(a4*a29);
  a69=(a60*a69);
  a29=(a29+a69);
  a68=(a5*a68);
  a67=(a44*a67);
  a68=(a68-a67);
  a29=(a29+a68);
  a68=(a22*a29);
  a67=(a64*a24);
  a68=(a68+a67);
  a52=(a52+a68);
  a68=(a20*a52);
  a53=(a53-a68);
  a51=(a51-a53);
  a53=(a1*a51);
  a68=(a42*a43);
  a53=(a53-a68);
  a68=(a20*a45);
  a70=(a61*a70);
  a68=(a68+a70);
  a52=(a3*a52);
  a13=(a63*a13);
  a52=(a52+a13);
  a68=(a68-a52);
  a52=(a38*a68);
  a13=(a19*a15);
  a52=(a52+a13);
  a53=(a53-a52);
  if (res[0]!=0) res[0][34]=a53;
  if (res[0]!=0) res[0][35]=a65;
  a71=(a22*a71);
  a53=(a6*a24);
  a71=(a71+a53);
  a53=(a30*a72);
  a52=(a26*a39);
  a53=(a53-a52);
  a71=(a71-a53);
  if (res[0]!=0) res[0][36]=a71;
  a53=(a30*a14);
  a52=(a50*a39);
  a53=(a53+a52);
  a10=(a22*a10);
  a52=(a54*a24);
  a10=(a10+a52);
  a53=(a53+a10);
  a10=(-a53);
  if (res[0]!=0) res[0][37]=a10;
  a10=(a30*a29);
  a39=(a64*a39);
  a10=(a10-a39);
  a24=(a62*a24);
  a59=(a22*a59);
  a24=(a24-a59);
  a10=(a10+a24);
  if (res[0]!=0) res[0][38]=a10;
  if (res[0]!=0) res[0][39]=a65;
  a24=(a34*a15);
  a66=(a38*a66);
  a24=(a24-a66);
  a66=(a1*a9);
  a59=(a40*a43);
  a66=(a66-a59);
  a24=(a24+a66);
  if (res[0]!=0) res[0][40]=a24;
  a8=(a38*a8);
  a66=(a56*a15);
  a8=(a8+a66);
  a66=(a1*a57);
  a59=(a58*a43);
  a66=(a66-a59);
  a8=(a8+a66);
  if (res[0]!=0) res[0][41]=a8;
  a51=(a38*a51);
  a15=(a42*a15);
  a51=(a51+a15);
  a15=(a1*a68);
  a43=(a19*a43);
  a15=(a15-a43);
  a51=(a51+a15);
  if (res[0]!=0) res[0][42]=a51;
  if (res[0]!=0) res[0][43]=a65;
  a24=(a36*a24);
  a9=(a36*a9);
  a71=(a37*a71);
  a15=arg[2]? arg[2][18] : 0;
  a18=(a41*a18);
  a72=(a28*a72);
  a18=(a18+a72);
  a15=(a15-a18);
  a71=(a71+a15);
  a9=(a9+a71);
  a24=(a24+a9);
  if (res[0]!=0) res[0][44]=a24;
  a8=(a36*a8);
  a57=(a36*a57);
  a48=(a41*a48);
  a14=(a28*a14);
  a48=(a48-a14);
  a14=arg[2]? arg[2][19] : 0;
  a48=(a48+a14);
  a53=(a37*a53);
  a48=(a48-a53);
  a57=(a57+a48);
  a8=(a8+a57);
  if (res[0]!=0) res[0][45]=a8;
  a51=(a36*a51);
  a68=(a36*a68);
  a10=(a37*a10);
  a45=(a41*a45);
  a29=(a28*a29);
  a45=(a45+a29);
  a29=arg[2]? arg[2][20] : 0;
  a45=(a45+a29);
  a10=(a10+a45);
  a68=(a68+a10);
  a51=(a51+a68);
  if (res[0]!=0) res[0][46]=a51;
  if (res[0]!=0) res[0][47]=a65;
  a51=arg[2]? arg[2][31] : 0;
  a68=(a7*a51);
  a10=(a5*a68);
  a45=arg[2]? arg[2][32] : 0;
  a29=(a12*a45);
  a8=(a11*a29);
  a10=(a10+a8);
  a8=(a3*a10);
  a57=arg[2]? arg[2][34] : 0;
  a48=(a17*a57);
  a53=(a16*a48);
  a8=(a8+a53);
  a45=(a23*a45);
  a53=(a11*a45);
  a14=(a4*a68);
  a53=(a53-a14);
  a14=(a22*a53);
  a24=arg[2]? arg[2][33] : 0;
  a9=(a27*a24);
  a71=(a26*a9);
  a14=(a14+a71);
  a51=(a31*a51);
  a71=(a30*a51);
  a24=(a21*a24);
  a15=(a6*a24);
  a71=(a71-a15);
  a14=(a14+a71);
  a71=(a20*a14);
  a57=(a2*a57);
  a15=(a33*a57);
  a71=(a71+a15);
  a8=(a8+a71);
  a71=(a1*a8);
  a15=arg[2]? arg[2][35] : 0;
  a18=(a35*a15);
  a72=(a34*a18);
  a71=(a71+a72);
  a72=(a16*a57);
  a43=(a20*a10);
  a72=(a72-a43);
  a14=(a3*a14);
  a43=(a33*a48);
  a14=(a14-a43);
  a72=(a72+a14);
  a14=(a38*a72);
  a15=(a0*a15);
  a43=(a40*a15);
  a14=(a14+a43);
  a71=(a71+a14);
  a71=(-a71);
  if (res[0]!=0) res[0][48]=a71;
  a71=arg[2]? arg[2][30] : 0;
  a14=(a25*a71);
  a43=(a6*a14);
  a66=(a44*a51);
  a43=(a43+a66);
  a66=(a5*a43);
  a59=(a46*a29);
  a66=(a66-a59);
  a59=(a47*a45);
  a71=(a32*a71);
  a39=(a4*a71);
  a59=(a59-a39);
  a66=(a66+a59);
  a59=(a3*a66);
  a39=(a49*a48);
  a59=(a59-a39);
  a39=(a50*a9);
  a52=(a5*a71);
  a13=(a47*a29);
  a52=(a52+a13);
  a43=(a4*a43);
  a13=(a46*a45);
  a43=(a43+a13);
  a52=(a52+a43);
  a43=(a22*a52);
  a39=(a39-a43);
  a43=(a11*a14);
  a13=(a44*a68);
  a43=(a43-a13);
  a13=(a30*a43);
  a70=(a54*a24);
  a13=(a13-a70);
  a39=(a39+a13);
  a13=(a20*a39);
  a70=(a55*a57);
  a13=(a13+a70);
  a59=(a59+a13);
  a13=(a1*a59);
  a70=(a56*a18);
  a13=(a13-a70);
  a70=(a20*a66);
  a67=(a49*a57);
  a70=(a70+a67);
  a39=(a3*a39);
  a67=(a55*a48);
  a39=(a39-a67);
  a70=(a70-a39);
  a39=(a38*a70);
  a67=(a58*a15);
  a39=(a39+a67);
  a13=(a13-a39);
  if (res[0]!=0) res[0][49]=a13;
  a13=(a4*a14);
  a39=(a44*a45);
  a13=(a13+a39);
  a39=(a47*a51);
  a67=(a6*a71);
  a39=(a39-a67);
  a67=(a5*a39);
  a69=(a60*a29);
  a67=(a67-a69);
  a13=(a13-a67);
  a67=(a3*a13);
  a69=(a61*a48);
  a67=(a67-a69);
  a69=(a63*a57);
  a71=(a11*a71);
  a68=(a47*a68);
  a71=(a71+a68);
  a68=(a30*a71);
  a73=(a62*a24);
  a68=(a68+a73);
  a39=(a4*a39);
  a45=(a60*a45);
  a39=(a39+a45);
  a14=(a5*a14);
  a29=(a44*a29);
  a14=(a14-a29);
  a39=(a39+a14);
  a14=(a22*a39);
  a29=(a64*a9);
  a14=(a14+a29);
  a68=(a68+a14);
  a14=(a20*a68);
  a69=(a69-a14);
  a67=(a67-a69);
  a69=(a1*a67);
  a14=(a42*a18);
  a69=(a69-a14);
  a14=(a20*a13);
  a57=(a61*a57);
  a14=(a14+a57);
  a68=(a3*a68);
  a48=(a63*a48);
  a68=(a68+a48);
  a14=(a14-a68);
  a68=(a38*a14);
  a48=(a19*a15);
  a68=(a68+a48);
  a69=(a69-a68);
  if (res[0]!=0) res[0][50]=a69;
  if (res[0]!=0) res[0][51]=a65;
  a51=(a22*a51);
  a69=(a6*a9);
  a51=(a51+a69);
  a69=(a30*a53);
  a68=(a26*a24);
  a69=(a69-a68);
  a51=(a51-a69);
  if (res[0]!=0) res[0][52]=a51;
  a69=(a30*a52);
  a68=(a50*a24);
  a69=(a69+a68);
  a43=(a22*a43);
  a68=(a54*a9);
  a43=(a43+a68);
  a69=(a69+a43);
  a43=(-a69);
  if (res[0]!=0) res[0][53]=a43;
  a43=(a30*a39);
  a24=(a64*a24);
  a43=(a43-a24);
  a9=(a62*a9);
  a71=(a22*a71);
  a9=(a9-a71);
  a43=(a43+a9);
  if (res[0]!=0) res[0][54]=a43;
  if (res[0]!=0) res[0][55]=a65;
  a9=(a34*a15);
  a8=(a38*a8);
  a9=(a9-a8);
  a8=(a1*a72);
  a71=(a40*a18);
  a8=(a8-a71);
  a9=(a9+a8);
  if (res[0]!=0) res[0][56]=a9;
  a59=(a38*a59);
  a8=(a56*a15);
  a59=(a59+a8);
  a8=(a1*a70);
  a71=(a58*a18);
  a8=(a8-a71);
  a59=(a59+a8);
  if (res[0]!=0) res[0][57]=a59;
  a67=(a38*a67);
  a15=(a42*a15);
  a67=(a67+a15);
  a15=(a1*a14);
  a18=(a19*a18);
  a15=(a15-a18);
  a67=(a67+a15);
  if (res[0]!=0) res[0][58]=a67;
  if (res[0]!=0) res[0][59]=a65;
  a9=(a36*a9);
  a72=(a36*a72);
  a51=(a37*a51);
  a15=arg[2]? arg[2][27] : 0;
  a10=(a41*a10);
  a53=(a28*a53);
  a10=(a10+a53);
  a15=(a15-a10);
  a51=(a51+a15);
  a72=(a72+a51);
  a9=(a9+a72);
  if (res[0]!=0) res[0][60]=a9;
  a59=(a36*a59);
  a70=(a36*a70);
  a66=(a41*a66);
  a52=(a28*a52);
  a66=(a66-a52);
  a52=arg[2]? arg[2][28] : 0;
  a66=(a66+a52);
  a69=(a37*a69);
  a66=(a66-a69);
  a70=(a70+a66);
  a59=(a59+a70);
  if (res[0]!=0) res[0][61]=a59;
  a67=(a36*a67);
  a14=(a36*a14);
  a43=(a37*a43);
  a13=(a41*a13);
  a39=(a28*a39);
  a13=(a13+a39);
  a39=arg[2]? arg[2][29] : 0;
  a13=(a13+a39);
  a43=(a43+a13);
  a14=(a14+a43);
  a67=(a67+a14);
  if (res[0]!=0) res[0][62]=a67;
  if (res[0]!=0) res[0][63]=a65;
  a67=arg[2]? arg[2][40] : 0;
  a14=(a7*a67);
  a43=(a5*a14);
  a13=arg[2]? arg[2][41] : 0;
  a39=(a12*a13);
  a59=(a11*a39);
  a43=(a43+a59);
  a59=(a3*a43);
  a70=arg[2]? arg[2][43] : 0;
  a66=(a17*a70);
  a69=(a16*a66);
  a59=(a59+a69);
  a13=(a23*a13);
  a69=(a11*a13);
  a52=(a4*a14);
  a69=(a69-a52);
  a52=(a22*a69);
  a9=arg[2]? arg[2][42] : 0;
  a72=(a27*a9);
  a51=(a26*a72);
  a52=(a52+a51);
  a67=(a31*a67);
  a51=(a30*a67);
  a9=(a21*a9);
  a15=(a6*a9);
  a51=(a51-a15);
  a52=(a52+a51);
  a51=(a20*a52);
  a70=(a2*a70);
  a15=(a33*a70);
  a51=(a51+a15);
  a59=(a59+a51);
  a51=(a1*a59);
  a15=arg[2]? arg[2][44] : 0;
  a10=(a35*a15);
  a53=(a34*a10);
  a51=(a51+a53);
  a53=(a16*a70);
  a18=(a20*a43);
  a53=(a53-a18);
  a52=(a3*a52);
  a18=(a33*a66);
  a52=(a52-a18);
  a53=(a53+a52);
  a52=(a38*a53);
  a15=(a0*a15);
  a18=(a40*a15);
  a52=(a52+a18);
  a51=(a51+a52);
  a51=(-a51);
  if (res[0]!=0) res[0][64]=a51;
  a51=arg[2]? arg[2][39] : 0;
  a52=(a25*a51);
  a18=(a6*a52);
  a8=(a44*a67);
  a18=(a18+a8);
  a8=(a5*a18);
  a71=(a46*a39);
  a8=(a8-a71);
  a71=(a47*a13);
  a51=(a32*a51);
  a24=(a4*a51);
  a71=(a71-a24);
  a8=(a8+a71);
  a71=(a3*a8);
  a24=(a49*a66);
  a71=(a71-a24);
  a24=(a50*a72);
  a68=(a5*a51);
  a48=(a47*a39);
  a68=(a68+a48);
  a18=(a4*a18);
  a48=(a46*a13);
  a18=(a18+a48);
  a68=(a68+a18);
  a18=(a22*a68);
  a24=(a24-a18);
  a18=(a11*a52);
  a48=(a44*a14);
  a18=(a18-a48);
  a48=(a30*a18);
  a57=(a54*a9);
  a48=(a48-a57);
  a24=(a24+a48);
  a48=(a20*a24);
  a57=(a55*a70);
  a48=(a48+a57);
  a71=(a71+a48);
  a48=(a1*a71);
  a57=(a56*a10);
  a48=(a48-a57);
  a57=(a20*a8);
  a29=(a49*a70);
  a57=(a57+a29);
  a24=(a3*a24);
  a29=(a55*a66);
  a24=(a24-a29);
  a57=(a57-a24);
  a24=(a38*a57);
  a29=(a58*a15);
  a24=(a24+a29);
  a48=(a48-a24);
  if (res[0]!=0) res[0][65]=a48;
  a48=(a4*a52);
  a24=(a44*a13);
  a48=(a48+a24);
  a24=(a47*a67);
  a29=(a6*a51);
  a24=(a24-a29);
  a29=(a5*a24);
  a45=(a60*a39);
  a29=(a29-a45);
  a48=(a48-a29);
  a29=(a3*a48);
  a45=(a61*a66);
  a29=(a29-a45);
  a45=(a63*a70);
  a51=(a11*a51);
  a14=(a47*a14);
  a51=(a51+a14);
  a14=(a30*a51);
  a73=(a62*a9);
  a14=(a14+a73);
  a24=(a4*a24);
  a13=(a60*a13);
  a24=(a24+a13);
  a52=(a5*a52);
  a39=(a44*a39);
  a52=(a52-a39);
  a24=(a24+a52);
  a52=(a22*a24);
  a39=(a64*a72);
  a52=(a52+a39);
  a14=(a14+a52);
  a52=(a20*a14);
  a45=(a45-a52);
  a29=(a29-a45);
  a45=(a1*a29);
  a52=(a42*a10);
  a45=(a45-a52);
  a52=(a20*a48);
  a70=(a61*a70);
  a52=(a52+a70);
  a14=(a3*a14);
  a66=(a63*a66);
  a14=(a14+a66);
  a52=(a52-a14);
  a14=(a38*a52);
  a66=(a19*a15);
  a14=(a14+a66);
  a45=(a45-a14);
  if (res[0]!=0) res[0][66]=a45;
  if (res[0]!=0) res[0][67]=a65;
  a67=(a22*a67);
  a45=(a6*a72);
  a67=(a67+a45);
  a45=(a30*a69);
  a14=(a26*a9);
  a45=(a45-a14);
  a67=(a67-a45);
  if (res[0]!=0) res[0][68]=a67;
  a45=(a30*a68);
  a14=(a50*a9);
  a45=(a45+a14);
  a18=(a22*a18);
  a14=(a54*a72);
  a18=(a18+a14);
  a45=(a45+a18);
  a18=(-a45);
  if (res[0]!=0) res[0][69]=a18;
  a18=(a30*a24);
  a9=(a64*a9);
  a18=(a18-a9);
  a72=(a62*a72);
  a51=(a22*a51);
  a72=(a72-a51);
  a18=(a18+a72);
  if (res[0]!=0) res[0][70]=a18;
  if (res[0]!=0) res[0][71]=a65;
  a72=(a34*a15);
  a59=(a38*a59);
  a72=(a72-a59);
  a59=(a1*a53);
  a51=(a40*a10);
  a59=(a59-a51);
  a72=(a72+a59);
  if (res[0]!=0) res[0][72]=a72;
  a71=(a38*a71);
  a59=(a56*a15);
  a71=(a71+a59);
  a59=(a1*a57);
  a51=(a58*a10);
  a59=(a59-a51);
  a71=(a71+a59);
  if (res[0]!=0) res[0][73]=a71;
  a29=(a38*a29);
  a15=(a42*a15);
  a29=(a29+a15);
  a15=(a1*a52);
  a10=(a19*a10);
  a15=(a15-a10);
  a29=(a29+a15);
  if (res[0]!=0) res[0][74]=a29;
  if (res[0]!=0) res[0][75]=a65;
  a72=(a36*a72);
  a53=(a36*a53);
  a67=(a37*a67);
  a15=arg[2]? arg[2][36] : 0;
  a43=(a41*a43);
  a69=(a28*a69);
  a43=(a43+a69);
  a15=(a15-a43);
  a67=(a67+a15);
  a53=(a53+a67);
  a72=(a72+a53);
  if (res[0]!=0) res[0][76]=a72;
  a71=(a36*a71);
  a57=(a36*a57);
  a8=(a41*a8);
  a68=(a28*a68);
  a8=(a8-a68);
  a68=arg[2]? arg[2][37] : 0;
  a8=(a8+a68);
  a45=(a37*a45);
  a8=(a8-a45);
  a57=(a57+a8);
  a71=(a71+a57);
  if (res[0]!=0) res[0][77]=a71;
  a29=(a36*a29);
  a52=(a36*a52);
  a18=(a37*a18);
  a48=(a41*a48);
  a24=(a28*a24);
  a48=(a48+a24);
  a24=arg[2]? arg[2][38] : 0;
  a48=(a48+a24);
  a18=(a18+a48);
  a52=(a52+a18);
  a29=(a29+a52);
  if (res[0]!=0) res[0][78]=a29;
  if (res[0]!=0) res[0][79]=a65;
  a29=arg[2]? arg[2][49] : 0;
  a52=(a7*a29);
  a18=(a5*a52);
  a48=arg[2]? arg[2][50] : 0;
  a24=(a12*a48);
  a71=(a11*a24);
  a18=(a18+a71);
  a71=(a3*a18);
  a57=arg[2]? arg[2][52] : 0;
  a8=(a17*a57);
  a45=(a16*a8);
  a71=(a71+a45);
  a48=(a23*a48);
  a45=(a11*a48);
  a68=(a4*a52);
  a45=(a45-a68);
  a68=(a22*a45);
  a72=arg[2]? arg[2][51] : 0;
  a53=(a27*a72);
  a67=(a26*a53);
  a68=(a68+a67);
  a29=(a31*a29);
  a67=(a30*a29);
  a72=(a21*a72);
  a15=(a6*a72);
  a67=(a67-a15);
  a68=(a68+a67);
  a67=(a20*a68);
  a57=(a2*a57);
  a15=(a33*a57);
  a67=(a67+a15);
  a71=(a71+a67);
  a67=(a1*a71);
  a15=arg[2]? arg[2][53] : 0;
  a43=(a35*a15);
  a69=(a34*a43);
  a67=(a67+a69);
  a69=(a16*a57);
  a10=(a20*a18);
  a69=(a69-a10);
  a68=(a3*a68);
  a10=(a33*a8);
  a68=(a68-a10);
  a69=(a69+a68);
  a68=(a38*a69);
  a15=(a0*a15);
  a10=(a40*a15);
  a68=(a68+a10);
  a67=(a67+a68);
  a67=(-a67);
  if (res[0]!=0) res[0][80]=a67;
  a67=arg[2]? arg[2][48] : 0;
  a68=(a25*a67);
  a10=(a6*a68);
  a59=(a44*a29);
  a10=(a10+a59);
  a59=(a5*a10);
  a51=(a46*a24);
  a59=(a59-a51);
  a51=(a47*a48);
  a67=(a32*a67);
  a9=(a4*a67);
  a51=(a51-a9);
  a59=(a59+a51);
  a51=(a3*a59);
  a9=(a49*a8);
  a51=(a51-a9);
  a9=(a50*a53);
  a14=(a5*a67);
  a66=(a47*a24);
  a14=(a14+a66);
  a10=(a4*a10);
  a66=(a46*a48);
  a10=(a10+a66);
  a14=(a14+a10);
  a10=(a22*a14);
  a9=(a9-a10);
  a10=(a11*a68);
  a66=(a44*a52);
  a10=(a10-a66);
  a66=(a30*a10);
  a70=(a54*a72);
  a66=(a66-a70);
  a9=(a9+a66);
  a66=(a20*a9);
  a70=(a55*a57);
  a66=(a66+a70);
  a51=(a51+a66);
  a66=(a1*a51);
  a70=(a56*a43);
  a66=(a66-a70);
  a70=(a20*a59);
  a39=(a49*a57);
  a70=(a70+a39);
  a9=(a3*a9);
  a39=(a55*a8);
  a9=(a9-a39);
  a70=(a70-a9);
  a9=(a38*a70);
  a39=(a58*a15);
  a9=(a9+a39);
  a66=(a66-a9);
  if (res[0]!=0) res[0][81]=a66;
  a66=(a4*a68);
  a9=(a44*a48);
  a66=(a66+a9);
  a9=(a47*a29);
  a39=(a6*a67);
  a9=(a9-a39);
  a39=(a5*a9);
  a13=(a60*a24);
  a39=(a39-a13);
  a66=(a66-a39);
  a39=(a3*a66);
  a13=(a61*a8);
  a39=(a39-a13);
  a13=(a63*a57);
  a67=(a11*a67);
  a52=(a47*a52);
  a67=(a67+a52);
  a52=(a30*a67);
  a73=(a62*a72);
  a52=(a52+a73);
  a9=(a4*a9);
  a48=(a60*a48);
  a9=(a9+a48);
  a68=(a5*a68);
  a24=(a44*a24);
  a68=(a68-a24);
  a9=(a9+a68);
  a68=(a22*a9);
  a24=(a64*a53);
  a68=(a68+a24);
  a52=(a52+a68);
  a68=(a20*a52);
  a13=(a13-a68);
  a39=(a39-a13);
  a13=(a1*a39);
  a68=(a42*a43);
  a13=(a13-a68);
  a68=(a20*a66);
  a57=(a61*a57);
  a68=(a68+a57);
  a52=(a3*a52);
  a8=(a63*a8);
  a52=(a52+a8);
  a68=(a68-a52);
  a52=(a38*a68);
  a8=(a19*a15);
  a52=(a52+a8);
  a13=(a13-a52);
  if (res[0]!=0) res[0][82]=a13;
  if (res[0]!=0) res[0][83]=a65;
  a29=(a22*a29);
  a13=(a6*a53);
  a29=(a29+a13);
  a13=(a30*a45);
  a52=(a26*a72);
  a13=(a13-a52);
  a29=(a29-a13);
  if (res[0]!=0) res[0][84]=a29;
  a13=(a30*a14);
  a52=(a50*a72);
  a13=(a13+a52);
  a10=(a22*a10);
  a52=(a54*a53);
  a10=(a10+a52);
  a13=(a13+a10);
  a10=(-a13);
  if (res[0]!=0) res[0][85]=a10;
  a10=(a30*a9);
  a72=(a64*a72);
  a10=(a10-a72);
  a53=(a62*a53);
  a67=(a22*a67);
  a53=(a53-a67);
  a10=(a10+a53);
  if (res[0]!=0) res[0][86]=a10;
  if (res[0]!=0) res[0][87]=a65;
  a53=(a34*a15);
  a71=(a38*a71);
  a53=(a53-a71);
  a71=(a1*a69);
  a67=(a40*a43);
  a71=(a71-a67);
  a53=(a53+a71);
  if (res[0]!=0) res[0][88]=a53;
  a51=(a38*a51);
  a71=(a56*a15);
  a51=(a51+a71);
  a71=(a1*a70);
  a67=(a58*a43);
  a71=(a71-a67);
  a51=(a51+a71);
  if (res[0]!=0) res[0][89]=a51;
  a39=(a38*a39);
  a15=(a42*a15);
  a39=(a39+a15);
  a15=(a1*a68);
  a43=(a19*a43);
  a15=(a15-a43);
  a39=(a39+a15);
  if (res[0]!=0) res[0][90]=a39;
  if (res[0]!=0) res[0][91]=a65;
  a53=(a36*a53);
  a69=(a36*a69);
  a29=(a37*a29);
  a15=arg[2]? arg[2][45] : 0;
  a18=(a41*a18);
  a45=(a28*a45);
  a18=(a18+a45);
  a15=(a15-a18);
  a29=(a29+a15);
  a69=(a69+a29);
  a53=(a53+a69);
  if (res[0]!=0) res[0][92]=a53;
  a51=(a36*a51);
  a70=(a36*a70);
  a59=(a41*a59);
  a14=(a28*a14);
  a59=(a59-a14);
  a14=arg[2]? arg[2][46] : 0;
  a59=(a59+a14);
  a13=(a37*a13);
  a59=(a59-a13);
  a70=(a70+a59);
  a51=(a51+a70);
  if (res[0]!=0) res[0][93]=a51;
  a39=(a36*a39);
  a68=(a36*a68);
  a10=(a37*a10);
  a66=(a41*a66);
  a9=(a28*a9);
  a66=(a66+a9);
  a9=arg[2]? arg[2][47] : 0;
  a66=(a66+a9);
  a10=(a10+a66);
  a68=(a68+a10);
  a39=(a39+a68);
  if (res[0]!=0) res[0][94]=a39;
  if (res[0]!=0) res[0][95]=a65;
  a39=arg[2]? arg[2][58] : 0;
  a7=(a7*a39);
  a68=(a5*a7);
  a10=arg[2]? arg[2][59] : 0;
  a12=(a12*a10);
  a66=(a11*a12);
  a68=(a68+a66);
  a66=(a3*a68);
  a9=arg[2]? arg[2][61] : 0;
  a17=(a17*a9);
  a51=(a16*a17);
  a66=(a66+a51);
  a23=(a23*a10);
  a10=(a11*a23);
  a51=(a4*a7);
  a10=(a10-a51);
  a51=(a22*a10);
  a70=arg[2]? arg[2][60] : 0;
  a27=(a27*a70);
  a59=(a26*a27);
  a51=(a51+a59);
  a31=(a31*a39);
  a39=(a30*a31);
  a21=(a21*a70);
  a70=(a6*a21);
  a39=(a39-a70);
  a51=(a51+a39);
  a39=(a20*a51);
  a2=(a2*a9);
  a9=(a33*a2);
  a39=(a39+a9);
  a66=(a66+a39);
  a39=(a1*a66);
  a9=arg[2]? arg[2][62] : 0;
  a35=(a35*a9);
  a70=(a34*a35);
  a39=(a39+a70);
  a16=(a16*a2);
  a70=(a20*a68);
  a16=(a16-a70);
  a51=(a3*a51);
  a33=(a33*a17);
  a51=(a51-a33);
  a16=(a16+a51);
  a51=(a38*a16);
  a0=(a0*a9);
  a9=(a40*a0);
  a51=(a51+a9);
  a39=(a39+a51);
  a39=(-a39);
  if (res[0]!=0) res[0][96]=a39;
  a39=arg[2]? arg[2][57] : 0;
  a25=(a25*a39);
  a51=(a6*a25);
  a9=(a44*a31);
  a51=(a51+a9);
  a9=(a5*a51);
  a33=(a46*a12);
  a9=(a9-a33);
  a33=(a47*a23);
  a32=(a32*a39);
  a39=(a4*a32);
  a33=(a33-a39);
  a9=(a9+a33);
  a33=(a3*a9);
  a39=(a49*a17);
  a33=(a33-a39);
  a39=(a50*a27);
  a70=(a5*a32);
  a59=(a47*a12);
  a70=(a70+a59);
  a51=(a4*a51);
  a46=(a46*a23);
  a51=(a51+a46);
  a70=(a70+a51);
  a51=(a22*a70);
  a39=(a39-a51);
  a51=(a11*a25);
  a46=(a44*a7);
  a51=(a51-a46);
  a46=(a30*a51);
  a59=(a54*a21);
  a46=(a46-a59);
  a39=(a39+a46);
  a46=(a20*a39);
  a59=(a55*a2);
  a46=(a46+a59);
  a33=(a33+a46);
  a46=(a1*a33);
  a59=(a56*a35);
  a46=(a46-a59);
  a59=(a20*a9);
  a49=(a49*a2);
  a59=(a59+a49);
  a39=(a3*a39);
  a55=(a55*a17);
  a39=(a39-a55);
  a59=(a59-a39);
  a39=(a38*a59);
  a55=(a58*a0);
  a39=(a39+a55);
  a46=(a46-a39);
  if (res[0]!=0) res[0][97]=a46;
  a46=(a4*a25);
  a39=(a44*a23);
  a46=(a46+a39);
  a39=(a47*a31);
  a55=(a6*a32);
  a39=(a39-a55);
  a55=(a5*a39);
  a49=(a60*a12);
  a55=(a55-a49);
  a46=(a46-a55);
  a55=(a3*a46);
  a49=(a61*a17);
  a55=(a55-a49);
  a49=(a63*a2);
  a11=(a11*a32);
  a47=(a47*a7);
  a11=(a11+a47);
  a47=(a30*a11);
  a7=(a62*a21);
  a47=(a47+a7);
  a4=(a4*a39);
  a60=(a60*a23);
  a4=(a4+a60);
  a5=(a5*a25);
  a44=(a44*a12);
  a5=(a5-a44);
  a4=(a4+a5);
  a5=(a22*a4);
  a44=(a64*a27);
  a5=(a5+a44);
  a47=(a47+a5);
  a5=(a20*a47);
  a49=(a49-a5);
  a55=(a55-a49);
  a49=(a1*a55);
  a5=(a42*a35);
  a49=(a49-a5);
  a20=(a20*a46);
  a61=(a61*a2);
  a20=(a20+a61);
  a3=(a3*a47);
  a63=(a63*a17);
  a3=(a3+a63);
  a20=(a20-a3);
  a3=(a38*a20);
  a63=(a19*a0);
  a3=(a3+a63);
  a49=(a49-a3);
  if (res[0]!=0) res[0][98]=a49;
  if (res[0]!=0) res[0][99]=a65;
  a31=(a22*a31);
  a6=(a6*a27);
  a31=(a31+a6);
  a6=(a30*a10);
  a26=(a26*a21);
  a6=(a6-a26);
  a31=(a31-a6);
  if (res[0]!=0) res[0][100]=a31;
  a6=(a30*a70);
  a50=(a50*a21);
  a6=(a6+a50);
  a51=(a22*a51);
  a54=(a54*a27);
  a51=(a51+a54);
  a6=(a6+a51);
  a51=(-a6);
  if (res[0]!=0) res[0][101]=a51;
  a30=(a30*a4);
  a64=(a64*a21);
  a30=(a30-a64);
  a62=(a62*a27);
  a22=(a22*a11);
  a62=(a62-a22);
  a30=(a30+a62);
  if (res[0]!=0) res[0][102]=a30;
  if (res[0]!=0) res[0][103]=a65;
  a34=(a34*a0);
  a66=(a38*a66);
  a34=(a34-a66);
  a66=(a1*a16);
  a40=(a40*a35);
  a66=(a66-a40);
  a34=(a34+a66);
  if (res[0]!=0) res[0][104]=a34;
  a33=(a38*a33);
  a56=(a56*a0);
  a33=(a33+a56);
  a56=(a1*a59);
  a58=(a58*a35);
  a56=(a56-a58);
  a33=(a33+a56);
  if (res[0]!=0) res[0][105]=a33;
  a38=(a38*a55);
  a42=(a42*a0);
  a38=(a38+a42);
  a1=(a1*a20);
  a19=(a19*a35);
  a1=(a1-a19);
  a38=(a38+a1);
  if (res[0]!=0) res[0][106]=a38;
  if (res[0]!=0) res[0][107]=a65;
  a34=(a36*a34);
  a16=(a36*a16);
  a31=(a37*a31);
  a1=arg[2]? arg[2][54] : 0;
  a68=(a41*a68);
  a10=(a28*a10);
  a68=(a68+a10);
  a1=(a1-a68);
  a31=(a31+a1);
  a16=(a16+a31);
  a34=(a34+a16);
  if (res[0]!=0) res[0][108]=a34;
  a33=(a36*a33);
  a59=(a36*a59);
  a9=(a41*a9);
  a70=(a28*a70);
  a9=(a9-a70);
  a70=arg[2]? arg[2][55] : 0;
  a9=(a9+a70);
  a6=(a37*a6);
  a9=(a9-a6);
  a59=(a59+a9);
  a33=(a33+a59);
  if (res[0]!=0) res[0][109]=a33;
  a38=(a36*a38);
  a36=(a36*a20);
  a37=(a37*a30);
  a41=(a41*a46);
  a28=(a28*a4);
  a41=(a41+a28);
  a28=arg[2]? arg[2][56] : 0;
  a41=(a41+a28);
  a37=(a37+a41);
  a36=(a36+a37);
  a38=(a38+a36);
  if (res[0]!=0) res[0][110]=a38;
  if (res[0]!=0) res[0][111]=a65;
  return 0;
}

CASADI_SYMBOL_EXPORT int fwd7_T_fk(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int fwd7_T_fk_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int fwd7_T_fk_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fwd7_T_fk_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int fwd7_T_fk_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fwd7_T_fk_release(int mem) {
}

CASADI_SYMBOL_EXPORT void fwd7_T_fk_incref(void) {
}

CASADI_SYMBOL_EXPORT void fwd7_T_fk_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int fwd7_T_fk_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int fwd7_T_fk_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real fwd7_T_fk_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fwd7_T_fk_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "out_o0";
    case 2: return "fwd_i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fwd7_T_fk_name_out(casadi_int i){
  switch (i) {
    case 0: return "fwd_o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fwd7_T_fk_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fwd7_T_fk_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int fwd7_T_fk_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
