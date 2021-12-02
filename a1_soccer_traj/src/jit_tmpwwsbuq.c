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
static const casadi_int casadi_s2[23] = {4, 4, 0, 4, 8, 12, 16, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};

/* adj1_T_fk:(i0[9],out_o0[4x4,0nz],adj_o0[4x4])->(adj_i0[9]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a5, a6, a7, a8, a9;
  a0=arg[2]? arg[2][12] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a1=arg[2]? arg[2][13] : 0;
  if (res[0]!=0) res[0][1]=a1;
  a2=arg[2]? arg[2][14] : 0;
  if (res[0]!=0) res[0][2]=a2;
  a3=arg[0]? arg[0][3] : 0;
  a4=cos(a3);
  a5=arg[0]? arg[0][5] : 0;
  a6=cos(a5);
  a7=-4.7000000000000000e-002;
  a8=(a7*a2);
  a9=arg[0]? arg[0][6] : 0;
  a10=cos(a9);
  a11=-8.5050000000000001e-002;
  a12=(a11*a2);
  a13=arg[2]? arg[2][6] : 0;
  a12=(a12+a13);
  a13=(a10*a12);
  a8=(a8+a13);
  a13=sin(a9);
  a14=arg[0]? arg[0][7] : 0;
  a15=cos(a14);
  a16=-2.0000000000000001e-001;
  a17=(a16*a2);
  a18=arg[0]? arg[0][8] : 0;
  a19=cos(a18);
  a20=(a16*a2);
  a21=arg[2]? arg[2][10] : 0;
  a20=(a20+a21);
  a21=(a19*a20);
  a17=(a17+a21);
  a21=sin(a18);
  a22=arg[2]? arg[2][2] : 0;
  a23=(a21*a22);
  a17=(a17-a23);
  a23=(a15*a17);
  a24=sin(a14);
  a25=(a21*a20);
  a26=(a19*a22);
  a25=(a25+a26);
  a26=(a24*a25);
  a23=(a23-a26);
  a26=(a13*a23);
  a8=(a8-a26);
  a26=(a6*a8);
  a27=sin(a5);
  a28=1.8300000000000000e-001;
  a2=(a28*a2);
  a29=(a24*a17);
  a2=(a2+a29);
  a29=(a15*a25);
  a2=(a2+a29);
  a29=(a27*a2);
  a26=(a26+a29);
  a29=arg[0]? arg[0][4] : 0;
  a30=cos(a29);
  a31=(a16*a1);
  a32=arg[2]? arg[2][9] : 0;
  a31=(a31+a32);
  a32=(a21*a31);
  a33=arg[2]? arg[2][1] : 0;
  a34=(a19*a33);
  a32=(a32+a34);
  a34=(a24*a32);
  a35=(a16*a1);
  a36=(a19*a31);
  a35=(a35+a36);
  a36=(a21*a33);
  a35=(a35-a36);
  a36=(a15*a35);
  a34=(a34-a36);
  a36=(a10*a34);
  a37=(a11*a1);
  a38=arg[2]? arg[2][5] : 0;
  a37=(a37+a38);
  a38=(a13*a37);
  a36=(a36-a38);
  a38=(a30*a36);
  a26=(a26+a38);
  a38=sin(a29);
  a39=(a28*a1);
  a40=(a24*a35);
  a39=(a39+a40);
  a40=(a15*a32);
  a39=(a39+a40);
  a40=(a6*a39);
  a1=(a7*a1);
  a41=(a10*a37);
  a1=(a1+a41);
  a41=(a13*a34);
  a1=(a1+a41);
  a41=(a27*a1);
  a40=(a40-a41);
  a41=(a38*a40);
  a26=(a26+a41);
  a4=(a4*a26);
  a26=sin(a3);
  a41=(a13*a12);
  a42=(a10*a23);
  a41=(a41+a42);
  a42=(a30*a41);
  a43=(a27*a8);
  a44=(a6*a2);
  a43=(a43-a44);
  a44=(a38*a43);
  a42=(a42+a44);
  a44=(a6*a1);
  a42=(a42+a44);
  a44=(a27*a39);
  a42=(a42+a44);
  a26=(a26*a42);
  a4=(a4-a26);
  if (res[0]!=0) res[0][3]=a4;
  a4=cos(a29);
  a11=(a11*a0);
  a26=arg[2]? arg[2][4] : 0;
  a11=(a11+a26);
  a26=(a13*a11);
  a42=cos(a3);
  a43=(a42*a43);
  a26=(a26+a43);
  a3=sin(a3);
  a40=(a3*a40);
  a26=(a26+a40);
  a40=(a16*a0);
  a16=(a16*a0);
  a43=arg[2]? arg[2][8] : 0;
  a16=(a16+a43);
  a43=(a19*a16);
  a40=(a40+a43);
  a43=arg[2]? arg[2][0] : 0;
  a44=(a21*a43);
  a40=(a40-a44);
  a44=(a15*a40);
  a21=(a21*a16);
  a19=(a19*a43);
  a21=(a21+a19);
  a19=(a24*a21);
  a44=(a44-a19);
  a19=(a10*a44);
  a26=(a26+a19);
  a4=(a4*a26);
  a29=sin(a29);
  a41=(a42*a41);
  a36=(a3*a36);
  a41=(a41+a36);
  a36=(a13*a44);
  a7=(a7*a0);
  a26=(a10*a11);
  a7=(a7+a26);
  a36=(a36-a7);
  a7=(a27*a36);
  a41=(a41+a7);
  a28=(a28*a0);
  a0=(a24*a40);
  a28=(a28+a0);
  a0=(a15*a21);
  a28=(a28+a0);
  a0=(a6*a28);
  a41=(a41+a0);
  a29=(a29*a41);
  a4=(a4-a29);
  if (res[0]!=0) res[0][4]=a4;
  a4=cos(a5);
  a29=(a42*a38);
  a41=(a29*a8);
  a0=(a3*a2);
  a41=(a41+a0);
  a0=(a3*a38);
  a7=(a0*a1);
  a41=(a41-a7);
  a7=(a42*a39);
  a41=(a41+a7);
  a36=(a30*a36);
  a41=(a41+a36);
  a4=(a4*a41);
  a5=sin(a5);
  a8=(a3*a8);
  a2=(a29*a2);
  a8=(a8-a2);
  a1=(a42*a1);
  a8=(a8+a1);
  a39=(a0*a39);
  a8=(a8+a39);
  a28=(a30*a28);
  a8=(a8+a28);
  a5=(a5*a8);
  a4=(a4-a5);
  if (res[0]!=0) res[0][5]=a4;
  a4=cos(a9);
  a5=(a42*a30);
  a8=(a5*a12);
  a28=(a3*a30);
  a39=(a28*a37);
  a8=(a8-a39);
  a39=(a38*a11);
  a8=(a8+a39);
  a39=(a29*a27);
  a1=(a3*a6);
  a39=(a39+a1);
  a1=(a39*a23);
  a8=(a8-a1);
  a1=(a42*a6);
  a2=(a0*a27);
  a1=(a1-a2);
  a2=(a1*a34);
  a8=(a8+a2);
  a2=(a30*a27);
  a41=(a2*a44);
  a8=(a8+a41);
  a4=(a4*a8);
  a9=sin(a9);
  a12=(a39*a12);
  a37=(a1*a37);
  a12=(a12+a37);
  a11=(a2*a11);
  a12=(a12-a11);
  a23=(a5*a23);
  a12=(a12+a23);
  a34=(a28*a34);
  a12=(a12+a34);
  a44=(a38*a44);
  a12=(a12+a44);
  a9=(a9*a12);
  a4=(a4-a9);
  if (res[0]!=0) res[0][6]=a4;
  a4=cos(a14);
  a3=(a3*a27);
  a29=(a29*a6);
  a3=(a3-a29);
  a29=(a3*a17);
  a5=(a5*a10);
  a39=(a39*a13);
  a5=(a5-a39);
  a39=(a5*a25);
  a29=(a29-a39);
  a0=(a0*a6);
  a42=(a42*a27);
  a0=(a0+a42);
  a42=(a0*a35);
  a29=(a29+a42);
  a1=(a1*a13);
  a28=(a28*a10);
  a1=(a1+a28);
  a28=(a1*a32);
  a29=(a29+a28);
  a30=(a30*a6);
  a6=(a30*a40);
  a29=(a29+a6);
  a2=(a2*a13);
  a38=(a38*a10);
  a2=(a2+a38);
  a38=(a2*a21);
  a29=(a29-a38);
  a4=(a4*a29);
  a14=sin(a14);
  a17=(a5*a17);
  a25=(a3*a25);
  a17=(a17+a25);
  a35=(a1*a35);
  a17=(a17-a35);
  a32=(a0*a32);
  a17=(a17+a32);
  a40=(a2*a40);
  a17=(a17+a40);
  a21=(a30*a21);
  a17=(a17+a21);
  a14=(a14*a17);
  a4=(a4-a14);
  if (res[0]!=0) res[0][7]=a4;
  a4=cos(a18);
  a14=(a3*a15);
  a17=(a5*a24);
  a14=(a14-a17);
  a17=(a14*a20);
  a21=(a0*a15);
  a40=(a1*a24);
  a21=(a21+a40);
  a40=(a21*a31);
  a17=(a17+a40);
  a40=(a30*a15);
  a32=(a2*a24);
  a40=(a40-a32);
  a32=(a40*a16);
  a17=(a17+a32);
  a3=(a3*a24);
  a5=(a5*a15);
  a3=(a3+a5);
  a5=(a3*a22);
  a17=(a17-a5);
  a0=(a0*a24);
  a1=(a1*a15);
  a0=(a0-a1);
  a1=(a0*a33);
  a17=(a17-a1);
  a30=(a30*a24);
  a2=(a2*a15);
  a30=(a30+a2);
  a2=(a30*a43);
  a17=(a17-a2);
  a4=(a4*a17);
  a18=sin(a18);
  a3=(a3*a20);
  a0=(a0*a31);
  a3=(a3+a0);
  a30=(a30*a16);
  a3=(a3+a30);
  a14=(a14*a22);
  a3=(a3+a14);
  a21=(a21*a33);
  a3=(a3+a21);
  a40=(a40*a43);
  a3=(a3+a40);
  a18=(a18*a3);
  a4=(a4-a18);
  if (res[0]!=0) res[0][8]=a4;
  return 0;
}

CASADI_SYMBOL_EXPORT int adj1_T_fk(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int adj1_T_fk_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int adj1_T_fk_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void adj1_T_fk_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int adj1_T_fk_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void adj1_T_fk_release(int mem) {
}

CASADI_SYMBOL_EXPORT void adj1_T_fk_incref(void) {
}

CASADI_SYMBOL_EXPORT void adj1_T_fk_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int adj1_T_fk_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int adj1_T_fk_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real adj1_T_fk_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* adj1_T_fk_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "out_o0";
    case 2: return "adj_o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* adj1_T_fk_name_out(casadi_int i){
  switch (i) {
    case 0: return "adj_i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* adj1_T_fk_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* adj1_T_fk_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int adj1_T_fk_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
