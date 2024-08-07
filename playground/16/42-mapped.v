// Benchmark "adder" written by ABC on Wed Jul 17 20:32:48 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n150, new_n151, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n175, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n188, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n217, new_n218, new_n219, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n228, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n278, new_n279,
    new_n280, new_n281, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n353, new_n355, new_n356, new_n357, new_n359, new_n360, new_n361,
    new_n363, new_n364, new_n366, new_n367, new_n368, new_n370, new_n371;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[2] ), .o1(new_n99));
  nanb02aa1n12x5               g004(.a(\a[3] ), .b(new_n99), .out0(new_n100));
  nand02aa1d10x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nand42aa1n02x5               g006(.a(new_n100), .b(new_n101), .o1(new_n102));
  nand42aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nor042aa1n04x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nand22aa1n06x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  oai012aa1n06x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  nor042aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  oab012aa1n06x5               g012(.a(new_n107), .b(\a[4] ), .c(\b[3] ), .out0(new_n108));
  oai012aa1n09x5               g013(.a(new_n108), .b(new_n106), .c(new_n102), .o1(new_n109));
  nor022aa1n06x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  inv000aa1n02x5               g015(.a(new_n110), .o1(new_n111));
  aoi022aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n112));
  nor022aa1n04x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  aoi012aa1n02x7               g018(.a(new_n113), .b(\a[4] ), .c(\b[3] ), .o1(new_n114));
  aoi022aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n115));
  nor002aa1n10x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  norp02aa1n24x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nona22aa1n02x4               g022(.a(new_n115), .b(new_n116), .c(new_n117), .out0(new_n118));
  nano32aa1n03x7               g023(.a(new_n118), .b(new_n114), .c(new_n112), .d(new_n111), .out0(new_n119));
  oai022aa1d24x5               g024(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n120));
  nand42aa1n04x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n121), .b(new_n116), .c(new_n113), .o1(new_n122));
  nanp02aa1n04x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nand42aa1n02x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  nanp03aa1n02x5               g029(.a(new_n121), .b(new_n123), .c(new_n124), .o1(new_n125));
  norp02aa1n02x5               g030(.a(new_n117), .b(new_n110), .o1(new_n126));
  oai013aa1n03x5               g031(.a(new_n122), .b(new_n125), .c(new_n126), .d(new_n120), .o1(new_n127));
  xnrc02aa1n12x5               g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n127), .c(new_n119), .d(new_n109), .o1(new_n130));
  xnrc02aa1n12x5               g035(.a(\b[9] ), .b(\a[10] ), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n130), .c(new_n98), .out0(\s[10] ));
  norb02aa1n02x5               g038(.a(new_n101), .b(new_n107), .out0(new_n134));
  oai112aa1n02x5               g039(.a(new_n134), .b(new_n103), .c(new_n104), .d(new_n105), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[4] ), .b(\a[5] ), .o1(new_n136));
  nano22aa1n02x4               g041(.a(new_n110), .b(new_n123), .c(new_n136), .out0(new_n137));
  oai022aa1n02x5               g042(.a(\a[6] ), .b(\b[5] ), .c(\b[6] ), .d(\a[7] ), .o1(new_n138));
  nano22aa1n02x4               g043(.a(new_n138), .b(new_n121), .c(new_n124), .out0(new_n139));
  nanp03aa1n02x5               g044(.a(new_n139), .b(new_n137), .c(new_n114), .o1(new_n140));
  and003aa1n03x5               g045(.a(new_n123), .b(new_n124), .c(new_n121), .o(new_n141));
  oab012aa1n04x5               g046(.a(new_n120), .b(new_n110), .c(new_n117), .out0(new_n142));
  aoi022aa1n03x5               g047(.a(new_n142), .b(new_n141), .c(new_n121), .d(new_n120), .o1(new_n143));
  aoai13aa1n04x5               g048(.a(new_n143), .b(new_n140), .c(new_n135), .d(new_n108), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n132), .b(new_n97), .c(new_n144), .d(new_n129), .o1(new_n145));
  inv000aa1d42x5               g050(.a(\a[10] ), .o1(new_n146));
  inv000aa1d42x5               g051(.a(\b[9] ), .o1(new_n147));
  oaoi03aa1n02x5               g052(.a(new_n146), .b(new_n147), .c(new_n97), .o1(new_n148));
  nor002aa1n03x5               g053(.a(\b[10] ), .b(\a[11] ), .o1(new_n149));
  nand02aa1n06x5               g054(.a(\b[10] ), .b(\a[11] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n145), .c(new_n148), .out0(\s[11] ));
  nor042aa1d18x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  nand22aa1n06x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  aoai13aa1n02x5               g060(.a(new_n148), .b(new_n131), .c(new_n130), .d(new_n98), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n149), .b(new_n156), .c(new_n150), .o1(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n154), .c(new_n155), .out0(\s[12] ));
  nona23aa1n03x5               g063(.a(new_n155), .b(new_n150), .c(new_n149), .d(new_n153), .out0(new_n159));
  nor043aa1n03x5               g064(.a(new_n159), .b(new_n131), .c(new_n128), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n127), .c(new_n119), .d(new_n109), .o1(new_n161));
  tech160nm_fiaoi012aa1n04x5   g066(.a(new_n97), .b(new_n146), .c(new_n147), .o1(new_n162));
  nanb03aa1n03x5               g067(.a(new_n153), .b(new_n155), .c(new_n150), .out0(new_n163));
  oaih22aa1n04x5               g068(.a(new_n146), .b(new_n147), .c(\b[10] ), .d(\a[11] ), .o1(new_n164));
  aoi012aa1n06x5               g069(.a(new_n153), .b(new_n149), .c(new_n155), .o1(new_n165));
  oai013aa1n03x5               g070(.a(new_n165), .b(new_n163), .c(new_n162), .d(new_n164), .o1(new_n166));
  nanb02aa1n03x5               g071(.a(new_n166), .b(new_n161), .out0(new_n167));
  nor002aa1n04x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  nand42aa1n06x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  nano22aa1n02x4               g075(.a(new_n153), .b(new_n150), .c(new_n155), .out0(new_n171));
  nona22aa1n02x4               g076(.a(new_n171), .b(new_n164), .c(new_n162), .out0(new_n172));
  nano22aa1n02x4               g077(.a(new_n170), .b(new_n172), .c(new_n165), .out0(new_n173));
  aoi022aa1n02x5               g078(.a(new_n167), .b(new_n170), .c(new_n161), .d(new_n173), .o1(\s[13] ));
  aoi012aa1n02x5               g079(.a(new_n168), .b(new_n167), .c(new_n169), .o1(new_n175));
  xnrb03aa1n03x5               g080(.a(new_n175), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  tech160nm_finand02aa1n03p5x5 g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  nano23aa1n03x5               g083(.a(new_n168), .b(new_n177), .c(new_n178), .d(new_n169), .out0(new_n179));
  aoai13aa1n03x5               g084(.a(new_n179), .b(new_n166), .c(new_n144), .d(new_n160), .o1(new_n180));
  oa0012aa1n02x5               g085(.a(new_n178), .b(new_n177), .c(new_n168), .o(new_n181));
  nanb02aa1n03x5               g086(.a(new_n181), .b(new_n180), .out0(new_n182));
  nor002aa1n16x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  nand02aa1n08x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  nanb02aa1n18x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  inv000aa1d42x5               g090(.a(new_n185), .o1(new_n186));
  oaih22aa1n04x5               g091(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n187));
  aboi22aa1n03x5               g092(.a(new_n183), .b(new_n184), .c(new_n187), .d(new_n178), .out0(new_n188));
  aoi022aa1n02x5               g093(.a(new_n182), .b(new_n186), .c(new_n180), .d(new_n188), .o1(\s[15] ));
  nor042aa1n06x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  nand22aa1n04x5               g095(.a(\b[15] ), .b(\a[16] ), .o1(new_n191));
  nanb02aa1n03x5               g096(.a(new_n190), .b(new_n191), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n183), .c(new_n182), .d(new_n184), .o1(new_n193));
  aoai13aa1n02x5               g098(.a(new_n186), .b(new_n181), .c(new_n167), .d(new_n179), .o1(new_n194));
  nona22aa1n02x4               g099(.a(new_n194), .b(new_n192), .c(new_n183), .out0(new_n195));
  nanp02aa1n02x5               g100(.a(new_n193), .b(new_n195), .o1(\s[16] ));
  nano23aa1n02x4               g101(.a(new_n149), .b(new_n153), .c(new_n155), .d(new_n150), .out0(new_n197));
  nona22aa1n03x5               g102(.a(new_n179), .b(new_n185), .c(new_n192), .out0(new_n198));
  nano32aa1n03x7               g103(.a(new_n198), .b(new_n132), .c(new_n129), .d(new_n197), .out0(new_n199));
  nanp02aa1n02x5               g104(.a(new_n144), .b(new_n199), .o1(new_n200));
  aoi012aa1n06x5               g105(.a(new_n127), .b(new_n119), .c(new_n109), .o1(new_n201));
  nona23aa1n03x5               g106(.a(new_n178), .b(new_n169), .c(new_n168), .d(new_n177), .out0(new_n202));
  nor043aa1n03x5               g107(.a(new_n202), .b(new_n185), .c(new_n192), .o1(new_n203));
  nanp02aa1n03x5               g108(.a(new_n203), .b(new_n160), .o1(new_n204));
  nano22aa1n03x7               g109(.a(new_n190), .b(new_n184), .c(new_n191), .out0(new_n205));
  aoi012aa1n03x5               g110(.a(new_n183), .b(\a[14] ), .c(\b[13] ), .o1(new_n206));
  nand23aa1n03x5               g111(.a(new_n205), .b(new_n187), .c(new_n206), .o1(new_n207));
  aoi012aa1n09x5               g112(.a(new_n190), .b(new_n183), .c(new_n191), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(new_n207), .b(new_n208), .o1(new_n209));
  tech160nm_fiaoi012aa1n05x5   g114(.a(new_n209), .b(new_n166), .c(new_n203), .o1(new_n210));
  oai012aa1n12x5               g115(.a(new_n210), .b(new_n201), .c(new_n204), .o1(new_n211));
  xorc02aa1n02x5               g116(.a(\a[17] ), .b(\b[16] ), .out0(new_n212));
  inv000aa1n02x5               g117(.a(new_n208), .o1(new_n213));
  nona22aa1n02x4               g118(.a(new_n207), .b(new_n213), .c(new_n212), .out0(new_n214));
  aoi012aa1n02x5               g119(.a(new_n214), .b(new_n166), .c(new_n203), .o1(new_n215));
  aoi022aa1n02x5               g120(.a(new_n211), .b(new_n212), .c(new_n200), .d(new_n215), .o1(\s[17] ));
  inv040aa1d32x5               g121(.a(\a[18] ), .o1(new_n217));
  nor002aa1n10x5               g122(.a(\b[16] ), .b(\a[17] ), .o1(new_n218));
  aoi012aa1n02x5               g123(.a(new_n218), .b(new_n211), .c(new_n212), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[17] ), .c(new_n217), .out0(\s[18] ));
  inv000aa1d42x5               g125(.a(\a[17] ), .o1(new_n221));
  xroi22aa1d06x4               g126(.a(new_n221), .b(\b[16] ), .c(new_n217), .d(\b[17] ), .out0(new_n222));
  nanp02aa1n04x5               g127(.a(\b[17] ), .b(\a[18] ), .o1(new_n223));
  oab012aa1n03x5               g128(.a(new_n218), .b(\a[18] ), .c(\b[17] ), .out0(new_n224));
  norb02aa1n02x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  xorc02aa1n03x5               g130(.a(\a[19] ), .b(\b[18] ), .out0(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n225), .c(new_n211), .d(new_n222), .o1(new_n227));
  aoi112aa1n02x5               g132(.a(new_n226), .b(new_n225), .c(new_n211), .d(new_n222), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n227), .b(new_n228), .out0(\s[19] ));
  xnrc02aa1n02x5               g134(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  and002aa1n12x5               g136(.a(\b[19] ), .b(\a[20] ), .o(new_n232));
  nor002aa1n03x5               g137(.a(new_n232), .b(new_n231), .o1(new_n233));
  inv040aa1n04x5               g138(.a(new_n233), .o1(new_n234));
  inv000aa1d42x5               g139(.a(\a[19] ), .o1(new_n235));
  oaib12aa1n06x5               g140(.a(new_n227), .b(\b[18] ), .c(new_n235), .out0(new_n236));
  nanp02aa1n02x5               g141(.a(new_n236), .b(new_n234), .o1(new_n237));
  norp02aa1n04x5               g142(.a(\b[18] ), .b(\a[19] ), .o1(new_n238));
  nona22aa1n02x4               g143(.a(new_n227), .b(new_n234), .c(new_n238), .out0(new_n239));
  nanp02aa1n02x5               g144(.a(new_n237), .b(new_n239), .o1(\s[20] ));
  norb02aa1n03x5               g145(.a(new_n226), .b(new_n234), .out0(new_n241));
  nand02aa1d08x5               g146(.a(new_n241), .b(new_n222), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoi112aa1n06x5               g148(.a(new_n232), .b(new_n231), .c(\a[19] ), .d(\b[18] ), .o1(new_n244));
  oai012aa1n03x5               g149(.a(new_n223), .b(\b[18] ), .c(\a[19] ), .o1(new_n245));
  nona22aa1n09x5               g150(.a(new_n244), .b(new_n224), .c(new_n245), .out0(new_n246));
  aoib12aa1n09x5               g151(.a(new_n231), .b(new_n238), .c(new_n232), .out0(new_n247));
  nanp02aa1n02x5               g152(.a(new_n246), .b(new_n247), .o1(new_n248));
  tech160nm_fixorc02aa1n05x5   g153(.a(\a[21] ), .b(\b[20] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n211), .d(new_n243), .o1(new_n250));
  nanb03aa1n02x5               g155(.a(new_n249), .b(new_n246), .c(new_n247), .out0(new_n251));
  aoi012aa1n02x5               g156(.a(new_n251), .b(new_n211), .c(new_n243), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n250), .b(new_n252), .out0(\s[21] ));
  xnrc02aa1n02x5               g158(.a(\b[21] ), .b(\a[22] ), .out0(new_n254));
  inv000aa1d42x5               g159(.a(\a[21] ), .o1(new_n255));
  oaib12aa1n06x5               g160(.a(new_n250), .b(\b[20] ), .c(new_n255), .out0(new_n256));
  nanp02aa1n02x5               g161(.a(new_n256), .b(new_n254), .o1(new_n257));
  norp02aa1n02x5               g162(.a(\b[20] ), .b(\a[21] ), .o1(new_n258));
  nona22aa1n02x4               g163(.a(new_n250), .b(new_n254), .c(new_n258), .out0(new_n259));
  nanp02aa1n02x5               g164(.a(new_n257), .b(new_n259), .o1(\s[22] ));
  aoi013aa1n06x4               g165(.a(new_n213), .b(new_n205), .c(new_n206), .d(new_n187), .o1(new_n261));
  aoai13aa1n02x7               g166(.a(new_n261), .b(new_n198), .c(new_n172), .d(new_n165), .o1(new_n262));
  inv000aa1d42x5               g167(.a(\a[22] ), .o1(new_n263));
  xroi22aa1d04x5               g168(.a(new_n255), .b(\b[20] ), .c(new_n263), .d(\b[21] ), .out0(new_n264));
  and003aa1n02x5               g169(.a(new_n241), .b(new_n222), .c(new_n264), .o(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n262), .c(new_n144), .d(new_n199), .o1(new_n266));
  nanb02aa1n12x5               g171(.a(new_n254), .b(new_n249), .out0(new_n267));
  inv000aa1d42x5               g172(.a(\b[21] ), .o1(new_n268));
  tech160nm_fioaoi03aa1n03p5x5 g173(.a(new_n263), .b(new_n268), .c(new_n258), .o1(new_n269));
  aoai13aa1n12x5               g174(.a(new_n269), .b(new_n267), .c(new_n246), .d(new_n247), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  nand22aa1n04x5               g176(.a(new_n266), .b(new_n271), .o1(new_n272));
  xnrc02aa1n12x5               g177(.a(\b[22] ), .b(\a[23] ), .out0(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(new_n269), .b(new_n273), .o1(new_n275));
  aoi012aa1n02x5               g180(.a(new_n275), .b(new_n248), .c(new_n264), .o1(new_n276));
  aoi022aa1n02x5               g181(.a(new_n272), .b(new_n274), .c(new_n266), .d(new_n276), .o1(\s[23] ));
  norp02aa1n02x5               g182(.a(\b[22] ), .b(\a[23] ), .o1(new_n278));
  tech160nm_fixnrc02aa1n05x5   g183(.a(\b[23] ), .b(\a[24] ), .out0(new_n279));
  aoai13aa1n02x5               g184(.a(new_n279), .b(new_n278), .c(new_n272), .d(new_n274), .o1(new_n280));
  aoi112aa1n03x5               g185(.a(new_n278), .b(new_n279), .c(new_n272), .d(new_n274), .o1(new_n281));
  nanb02aa1n03x5               g186(.a(new_n281), .b(new_n280), .out0(\s[24] ));
  nor042aa1n12x5               g187(.a(new_n279), .b(new_n273), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  nona32aa1n03x5               g189(.a(new_n211), .b(new_n284), .c(new_n267), .d(new_n242), .out0(new_n285));
  inv000aa1d42x5               g190(.a(\a[24] ), .o1(new_n286));
  inv000aa1d42x5               g191(.a(\b[23] ), .o1(new_n287));
  oao003aa1n02x5               g192(.a(new_n286), .b(new_n287), .c(new_n278), .carry(new_n288));
  aoi012aa1d18x5               g193(.a(new_n288), .b(new_n270), .c(new_n283), .o1(new_n289));
  nanp02aa1n03x5               g194(.a(new_n285), .b(new_n289), .o1(new_n290));
  tech160nm_fixorc02aa1n03p5x5 g195(.a(\a[25] ), .b(\b[24] ), .out0(new_n291));
  aoi112aa1n02x5               g196(.a(new_n291), .b(new_n288), .c(new_n270), .d(new_n283), .o1(new_n292));
  aoi022aa1n02x5               g197(.a(new_n290), .b(new_n291), .c(new_n285), .d(new_n292), .o1(\s[25] ));
  nor042aa1n03x5               g198(.a(\b[24] ), .b(\a[25] ), .o1(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[25] ), .b(\a[26] ), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n294), .c(new_n290), .d(new_n291), .o1(new_n296));
  inv020aa1n02x5               g201(.a(new_n289), .o1(new_n297));
  oaoi13aa1n02x5               g202(.a(new_n284), .b(new_n210), .c(new_n201), .d(new_n204), .o1(new_n298));
  aoai13aa1n02x5               g203(.a(new_n291), .b(new_n297), .c(new_n298), .d(new_n265), .o1(new_n299));
  nona22aa1n02x4               g204(.a(new_n299), .b(new_n295), .c(new_n294), .out0(new_n300));
  nanp02aa1n03x5               g205(.a(new_n296), .b(new_n300), .o1(\s[26] ));
  norb02aa1n02x5               g206(.a(new_n291), .b(new_n295), .out0(new_n302));
  aoai13aa1n12x5               g207(.a(new_n302), .b(new_n288), .c(new_n270), .d(new_n283), .o1(new_n303));
  nano32aa1n03x7               g208(.a(new_n242), .b(new_n302), .c(new_n264), .d(new_n283), .out0(new_n304));
  aoai13aa1n04x5               g209(.a(new_n304), .b(new_n262), .c(new_n144), .d(new_n199), .o1(new_n305));
  inv000aa1n03x5               g210(.a(new_n294), .o1(new_n306));
  oaoi03aa1n02x5               g211(.a(\a[26] ), .b(\b[25] ), .c(new_n306), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n307), .o1(new_n308));
  nand23aa1n06x5               g213(.a(new_n303), .b(new_n305), .c(new_n308), .o1(new_n309));
  xorc02aa1n12x5               g214(.a(\a[27] ), .b(\b[26] ), .out0(new_n310));
  aoi112aa1n02x5               g215(.a(new_n310), .b(new_n307), .c(new_n211), .d(new_n304), .o1(new_n311));
  aoi022aa1n02x5               g216(.a(new_n309), .b(new_n310), .c(new_n311), .d(new_n303), .o1(\s[27] ));
  norp02aa1n02x5               g217(.a(\b[26] ), .b(\a[27] ), .o1(new_n313));
  norp02aa1n02x5               g218(.a(\b[27] ), .b(\a[28] ), .o1(new_n314));
  nand02aa1n03x5               g219(.a(\b[27] ), .b(\a[28] ), .o1(new_n315));
  nanb02aa1n06x5               g220(.a(new_n314), .b(new_n315), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n313), .c(new_n309), .d(new_n310), .o1(new_n317));
  nand42aa1n02x5               g222(.a(new_n309), .b(new_n310), .o1(new_n318));
  nona22aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n313), .out0(new_n319));
  nanp02aa1n03x5               g224(.a(new_n319), .b(new_n317), .o1(\s[28] ));
  norb02aa1n03x5               g225(.a(new_n310), .b(new_n316), .out0(new_n321));
  nanp02aa1n03x5               g226(.a(new_n309), .b(new_n321), .o1(new_n322));
  aoi012aa1n03x5               g227(.a(new_n307), .b(new_n211), .c(new_n304), .o1(new_n323));
  inv000aa1d42x5               g228(.a(new_n321), .o1(new_n324));
  aoi012aa1n02x5               g229(.a(new_n314), .b(new_n313), .c(new_n315), .o1(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n324), .c(new_n323), .d(new_n303), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[29] ), .b(\b[28] ), .out0(new_n327));
  norb02aa1n02x5               g232(.a(new_n325), .b(new_n327), .out0(new_n328));
  aoi022aa1n03x5               g233(.a(new_n326), .b(new_n327), .c(new_n322), .d(new_n328), .o1(\s[29] ));
  xorb03aa1n02x5               g234(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g235(.a(new_n316), .b(new_n327), .c(new_n310), .out0(new_n331));
  nanb02aa1n03x5               g236(.a(new_n331), .b(new_n309), .out0(new_n332));
  inv000aa1d42x5               g237(.a(\b[28] ), .o1(new_n333));
  inv000aa1d42x5               g238(.a(\a[29] ), .o1(new_n334));
  oaib12aa1n02x5               g239(.a(new_n325), .b(\b[28] ), .c(new_n334), .out0(new_n335));
  oaib12aa1n02x5               g240(.a(new_n335), .b(new_n333), .c(\a[29] ), .out0(new_n336));
  aoai13aa1n02x5               g241(.a(new_n336), .b(new_n331), .c(new_n323), .d(new_n303), .o1(new_n337));
  xorc02aa1n02x5               g242(.a(\a[30] ), .b(\b[29] ), .out0(new_n338));
  oaoi13aa1n02x5               g243(.a(new_n338), .b(new_n335), .c(new_n334), .d(new_n333), .o1(new_n339));
  aoi022aa1n03x5               g244(.a(new_n337), .b(new_n338), .c(new_n332), .d(new_n339), .o1(\s[30] ));
  nanp03aa1n02x5               g245(.a(new_n321), .b(new_n327), .c(new_n338), .o1(new_n341));
  nanb02aa1n03x5               g246(.a(new_n341), .b(new_n309), .out0(new_n342));
  aoi022aa1n02x5               g247(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n343));
  norb02aa1n02x5               g248(.a(\b[30] ), .b(\a[31] ), .out0(new_n344));
  obai22aa1n02x7               g249(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n345));
  aoi112aa1n02x5               g250(.a(new_n345), .b(new_n344), .c(new_n335), .d(new_n343), .o1(new_n346));
  xorc02aa1n02x5               g251(.a(\a[31] ), .b(\b[30] ), .out0(new_n347));
  norp02aa1n02x5               g252(.a(\b[29] ), .b(\a[30] ), .o1(new_n348));
  aoi012aa1n02x5               g253(.a(new_n348), .b(new_n335), .c(new_n343), .o1(new_n349));
  aoai13aa1n02x5               g254(.a(new_n349), .b(new_n341), .c(new_n323), .d(new_n303), .o1(new_n350));
  aoi022aa1n03x5               g255(.a(new_n350), .b(new_n347), .c(new_n342), .d(new_n346), .o1(\s[31] ));
  xnbna2aa1n03x5               g256(.a(new_n106), .b(new_n100), .c(new_n101), .out0(\s[3] ));
  xorc02aa1n02x5               g257(.a(\a[4] ), .b(\b[3] ), .out0(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n353), .b(new_n135), .c(new_n100), .out0(\s[4] ));
  and002aa1n02x5               g259(.a(\b[3] ), .b(\a[4] ), .o(new_n355));
  aboi22aa1n03x5               g260(.a(new_n355), .b(new_n109), .c(new_n111), .d(new_n136), .out0(new_n356));
  aoi022aa1n02x5               g261(.a(\b[4] ), .b(\a[5] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n357));
  aoi013aa1n02x4               g262(.a(new_n356), .b(new_n111), .c(new_n109), .d(new_n357), .o1(\s[5] ));
  norb02aa1n02x5               g263(.a(new_n123), .b(new_n117), .out0(new_n359));
  aoai13aa1n02x5               g264(.a(new_n359), .b(new_n110), .c(new_n109), .d(new_n357), .o1(new_n360));
  aoi112aa1n02x5               g265(.a(new_n110), .b(new_n359), .c(new_n109), .d(new_n357), .o1(new_n361));
  norb02aa1n02x5               g266(.a(new_n360), .b(new_n361), .out0(\s[6] ));
  inv000aa1d42x5               g267(.a(new_n117), .o1(new_n363));
  norb02aa1n02x5               g268(.a(new_n124), .b(new_n116), .out0(new_n364));
  xnbna2aa1n03x5               g269(.a(new_n364), .b(new_n360), .c(new_n363), .out0(\s[7] ));
  inv000aa1d42x5               g270(.a(new_n116), .o1(new_n366));
  aob012aa1n02x5               g271(.a(new_n364), .b(new_n360), .c(new_n363), .out0(new_n367));
  norb02aa1n02x5               g272(.a(new_n121), .b(new_n113), .out0(new_n368));
  xnbna2aa1n03x5               g273(.a(new_n368), .b(new_n367), .c(new_n366), .out0(\s[8] ));
  nanp02aa1n02x5               g274(.a(new_n119), .b(new_n109), .o1(new_n370));
  aoi122aa1n02x5               g275(.a(new_n129), .b(new_n121), .c(new_n120), .d(new_n142), .e(new_n141), .o1(new_n371));
  aoi022aa1n02x5               g276(.a(new_n144), .b(new_n129), .c(new_n370), .d(new_n371), .o1(\s[9] ));
endmodule


