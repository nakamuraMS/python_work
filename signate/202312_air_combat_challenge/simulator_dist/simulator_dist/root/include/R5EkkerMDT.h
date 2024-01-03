// Copyright (c) 2021-2023 Air Systems Research Center, Acquisition, Technology & Logistics Agency(ATLA)
#pragma once
#include <utility>
/*
Based on [Ekker 1994] Appendix. B, but modified a bit.
(1) All values should be provided in MKS units. (angles should be in rad.)
(2) Transonic region (M=0.95 to 1.2) is also calculated by same method as either sub/supersonic region to simplify, although it is not so accuarate.
*/
namespace R5EkkerMDT{
    double areapogv(double d,double ln);
    double bdycla(double d,double l,double ln,double Mach,double alpha=0.0);
    double bigkbwa(double M,double A,double r,double s);
    double bigkbwna(double M,double A,double r,double s);
    double bigkbwsb(double r,double s);
    double bigkwb(double r,double s);
    double bkbwsba(double M,double A,double r,double s);
    double bkbwsbna(double M,double A,double r,double s);
    double bkbwspa(double M,double A,double r,double s);
    double bkbwspna(double M,double A,double r,double s);
    double bsdrgcf(double Mach);
    double bsdrgsp(double Mach);
    double cdbbody(double d,double l,double ln,double Mach,double atr);
    double cdlcomp(double bw,double Sw,double Mach);
    double cdlwing(double bw,double Sw,double Mach_,double atr);
    double cdobody(double d,double l,double ln,double Mach,double alt,double nf=1.1,bool pwroff=false);
    double cdowbt(double d,double l,double ln,double bw,double Sw,double thickw,double bt,double St,double thickt,double Mach,double alt,double nf=1.1,bool pwroff=false);
    double cdowing(double bw,double Sw,double thick,double Mach_,double alt,double nf=1.1);
    double cdtrim(double d,double l,double ln,double bw,double Sw,double thickw,double bt,double St,double thickt,double Mach,double alt,double atr,double dtr,double nf=1.1,bool pwroff=false);
    double cfturbfp(double chardim,double Mach,double alt);
    std::pair<double,double> clacma(double d,double l,double ln,double lcg,double bw,double Sw,double lw,double bt,double St,double Mach,double alpha=0.0);
    double clawsub(double M,double A,double lc2,double kappa);
    double clawsup(double M,double A);
    std::pair<double,double> cldcmd(double d,double l,double ln,double lcg,double bt,double St,double Mach);
    double cldwbt(double d,double bt,double St,double Mach);
    double cpogvemp(double d,double ln,double M);
    double cpogvsb(double ln,double d);
    double dedasub(double A,double b,double lc4,double lam,double lH,double hH);
    double dedasup(double M,double lam,double x0);
    double dragfctr(double fineness);
    double k2mk1(double fineness);
    double smlkbw(double r,double s);
    double smlkwb(double r,double s);
    double sscfdccc(double alpha,double M);
    double surfogv1(double diameter,double length);
    double vologv1(double logv,double dbase);
    double wvdrgogv(double diameter,double length,double Mach);
    double xcrbwabh(double M,double r,double cr);
    double xcrbwabl(double M,double A,double r,double s);
    double xcrbwnab(double M,double r,double cr);
    double xcrbwsub(double M,double A,double r,double s);
    double xcrw(double M,double A);
    double xcrwba(double r,double s);
    double xcrwbd(double r,double s);
    double xcrwbsub(double M,double A);
}