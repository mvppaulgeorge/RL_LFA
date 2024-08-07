// Benchmark "adder" written by ABC on Thu Jul 18 07:44:57 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n179, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n197, new_n198, new_n199, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n217, new_n218, new_n219, new_n220, new_n221, new_n222, new_n223,
    new_n224, new_n225, new_n226, new_n227, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n236, new_n237, new_n238, new_n239, new_n240,
    new_n241, new_n242, new_n243, new_n244, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n264, new_n265, new_n266, new_n267, new_n268, new_n269, new_n270,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n277, new_n278,
    new_n279, new_n280, new_n281, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n309,
    new_n310, new_n311, new_n312, new_n313, new_n314, new_n315, new_n317,
    new_n318, new_n319, new_n320, new_n321, new_n322, new_n323, new_n324,
    new_n325, new_n327, new_n328, new_n329, new_n330, new_n331, new_n332,
    new_n333, new_n334, new_n336, new_n337, new_n338, new_n339, new_n340,
    new_n341, new_n342, new_n345, new_n346, new_n347, new_n348, new_n349,
    new_n350, new_n351, new_n352, new_n353, new_n354, new_n356, new_n357,
    new_n358, new_n359, new_n360, new_n361, new_n362, new_n363, new_n364,
    new_n365, new_n366, new_n368, new_n370, new_n371, new_n373, new_n375,
    new_n376, new_n377, new_n379, new_n380, new_n382, new_n383, new_n384,
    new_n386;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  nor042aa1n06x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  inv000aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  nand22aa1n04x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nanp02aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aob012aa1n03x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nand02aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n03x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  nor042aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand42aa1n06x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n03x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nanp03aa1n06x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  tech160nm_fiaoi012aa1n05x5   g014(.a(new_n104), .b(new_n106), .c(new_n103), .o1(new_n110));
  nand02aa1n03x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nor002aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand42aa1n03x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nor002aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nano23aa1n03x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nor042aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand22aa1n04x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand02aa1n04x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nor042aa1d18x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n119), .b(new_n116), .c(new_n117), .d(new_n118), .out0(new_n120));
  nanp02aa1n03x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  norb02aa1n06x4               g026(.a(new_n117), .b(new_n116), .out0(new_n122));
  inv040aa1n02x5               g027(.a(new_n119), .o1(new_n123));
  oaoi03aa1n09x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  nano22aa1n03x7               g029(.a(new_n119), .b(new_n111), .c(new_n118), .out0(new_n125));
  oai022aa1n03x5               g030(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n126));
  aoi013aa1n06x4               g031(.a(new_n124), .b(new_n125), .c(new_n122), .d(new_n126), .o1(new_n127));
  aoai13aa1n12x5               g032(.a(new_n127), .b(new_n121), .c(new_n109), .d(new_n110), .o1(new_n128));
  norp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  xorc02aa1n12x5               g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoi112aa1n02x5               g035(.a(new_n129), .b(new_n97), .c(new_n128), .d(new_n130), .o1(new_n131));
  aoi012aa1n02x5               g036(.a(new_n98), .b(new_n100), .c(new_n101), .o1(new_n132));
  nona23aa1n03x5               g037(.a(new_n103), .b(new_n107), .c(new_n106), .d(new_n104), .out0(new_n133));
  oai012aa1n03x5               g038(.a(new_n110), .b(new_n133), .c(new_n132), .o1(new_n134));
  nanb02aa1n06x5               g039(.a(new_n121), .b(new_n134), .out0(new_n135));
  inv000aa1n02x5               g040(.a(new_n129), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n130), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n136), .b(new_n137), .c(new_n135), .d(new_n127), .o1(new_n138));
  aoi012aa1n02x5               g043(.a(new_n131), .b(new_n138), .c(new_n97), .o1(\s[10] ));
  orn002aa1n02x5               g044(.a(\a[10] ), .b(\b[9] ), .o(new_n140));
  nand42aa1n02x5               g045(.a(\b[9] ), .b(\a[10] ), .o1(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n129), .c(new_n128), .d(new_n130), .o1(new_n142));
  nand42aa1n02x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nor042aa1n06x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n142), .c(new_n140), .out0(\s[11] ));
  inv000aa1d42x5               g051(.a(new_n140), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n145), .b(new_n147), .c(new_n138), .d(new_n141), .o1(new_n148));
  norp02aa1n04x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nand02aa1n03x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  inv000aa1d42x5               g056(.a(\a[11] ), .o1(new_n152));
  inv000aa1d42x5               g057(.a(\b[10] ), .o1(new_n153));
  aboi22aa1n03x5               g058(.a(new_n149), .b(new_n150), .c(new_n152), .d(new_n153), .out0(new_n154));
  inv040aa1n03x5               g059(.a(new_n144), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n145), .o1(new_n156));
  aoai13aa1n03x5               g061(.a(new_n155), .b(new_n156), .c(new_n142), .d(new_n140), .o1(new_n157));
  aoi022aa1n02x7               g062(.a(new_n157), .b(new_n151), .c(new_n148), .d(new_n154), .o1(\s[12] ));
  nano23aa1n06x5               g063(.a(new_n149), .b(new_n144), .c(new_n150), .d(new_n143), .out0(new_n159));
  nand23aa1d12x5               g064(.a(new_n159), .b(new_n130), .c(new_n97), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(new_n128), .b(new_n161), .o1(new_n162));
  oaoi03aa1n09x5               g067(.a(\a[12] ), .b(\b[11] ), .c(new_n155), .o1(new_n163));
  oai022aa1n02x7               g068(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n164));
  nona22aa1n02x4               g069(.a(new_n150), .b(new_n149), .c(new_n144), .out0(new_n165));
  nano32aa1n02x4               g070(.a(new_n165), .b(new_n164), .c(new_n143), .d(new_n141), .out0(new_n166));
  nor042aa1n04x5               g071(.a(new_n166), .b(new_n163), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n160), .c(new_n135), .d(new_n127), .o1(new_n168));
  nand42aa1n16x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  nor002aa1n04x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  norp03aa1n02x5               g076(.a(new_n166), .b(new_n171), .c(new_n163), .o1(new_n172));
  aoi022aa1n02x5               g077(.a(new_n168), .b(new_n171), .c(new_n162), .d(new_n172), .o1(\s[13] ));
  orn002aa1n02x5               g078(.a(\a[13] ), .b(\b[12] ), .o(new_n174));
  inv000aa1d42x5               g079(.a(new_n167), .o1(new_n175));
  aoai13aa1n03x5               g080(.a(new_n171), .b(new_n175), .c(new_n128), .d(new_n161), .o1(new_n176));
  nor042aa1n02x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nand42aa1n08x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n176), .c(new_n174), .out0(\s[14] ));
  nano23aa1n03x7               g085(.a(new_n177), .b(new_n170), .c(new_n178), .d(new_n169), .out0(new_n181));
  aoai13aa1n06x5               g086(.a(new_n181), .b(new_n175), .c(new_n128), .d(new_n161), .o1(new_n182));
  oaoi03aa1n02x5               g087(.a(\a[14] ), .b(\b[13] ), .c(new_n174), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  xorc02aa1n12x5               g089(.a(\a[15] ), .b(\b[14] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n182), .c(new_n184), .out0(\s[15] ));
  aoai13aa1n02x5               g091(.a(new_n185), .b(new_n183), .c(new_n168), .d(new_n181), .o1(new_n187));
  xorc02aa1n02x5               g092(.a(\a[16] ), .b(\b[15] ), .out0(new_n188));
  inv000aa1d42x5               g093(.a(\a[15] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[14] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\a[16] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[15] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[15] ), .b(\a[16] ), .o1(new_n194));
  aoi022aa1n02x5               g099(.a(new_n193), .b(new_n194), .c(new_n190), .d(new_n189), .o1(new_n195));
  nor042aa1n06x5               g100(.a(\b[14] ), .b(\a[15] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  inv000aa1d42x5               g102(.a(new_n185), .o1(new_n198));
  aoai13aa1n02x5               g103(.a(new_n197), .b(new_n198), .c(new_n182), .d(new_n184), .o1(new_n199));
  aoi022aa1n03x5               g104(.a(new_n199), .b(new_n188), .c(new_n187), .d(new_n195), .o1(\s[16] ));
  nona23aa1n02x4               g105(.a(new_n169), .b(new_n178), .c(new_n177), .d(new_n170), .out0(new_n201));
  nano22aa1n06x5               g106(.a(new_n201), .b(new_n185), .c(new_n188), .out0(new_n202));
  nanb02aa1n03x5               g107(.a(new_n160), .b(new_n202), .out0(new_n203));
  oai112aa1n02x5               g108(.a(new_n193), .b(new_n194), .c(\b[14] ), .d(\a[15] ), .o1(new_n204));
  oai122aa1n06x5               g109(.a(new_n178), .b(new_n177), .c(new_n170), .d(new_n190), .e(new_n189), .o1(new_n205));
  oaoi03aa1n12x5               g110(.a(new_n191), .b(new_n192), .c(new_n196), .o1(new_n206));
  oai012aa1n02x5               g111(.a(new_n206), .b(new_n205), .c(new_n204), .o1(new_n207));
  oaoi13aa1n02x7               g112(.a(new_n207), .b(new_n202), .c(new_n166), .d(new_n163), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n203), .c(new_n135), .d(new_n127), .o1(new_n209));
  xorc02aa1n02x5               g114(.a(\a[17] ), .b(\b[16] ), .out0(new_n210));
  nand23aa1n03x5               g115(.a(new_n181), .b(new_n185), .c(new_n188), .o1(new_n211));
  nor042aa1n06x5               g116(.a(new_n211), .b(new_n160), .o1(new_n212));
  xnrc02aa1n02x5               g117(.a(\b[16] ), .b(\a[17] ), .out0(new_n213));
  oai112aa1n02x5               g118(.a(new_n206), .b(new_n213), .c(new_n205), .d(new_n204), .o1(new_n214));
  aoi122aa1n02x5               g119(.a(new_n214), .b(new_n175), .c(new_n202), .d(new_n128), .e(new_n212), .o1(new_n215));
  aoi012aa1n02x5               g120(.a(new_n215), .b(new_n209), .c(new_n210), .o1(\s[17] ));
  inv000aa1d42x5               g121(.a(\a[17] ), .o1(new_n217));
  nanb02aa1n02x5               g122(.a(\b[16] ), .b(new_n217), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n163), .o1(new_n219));
  oai022aa1n02x5               g124(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n220));
  nano32aa1n02x4               g125(.a(new_n220), .b(new_n150), .c(new_n143), .d(new_n141), .out0(new_n221));
  oaib12aa1n02x5               g126(.a(new_n221), .b(new_n164), .c(new_n141), .out0(new_n222));
  inv020aa1n02x5               g127(.a(new_n206), .o1(new_n223));
  oab012aa1n03x5               g128(.a(new_n223), .b(new_n205), .c(new_n204), .out0(new_n224));
  aoai13aa1n12x5               g129(.a(new_n224), .b(new_n211), .c(new_n222), .d(new_n219), .o1(new_n225));
  aoai13aa1n06x5               g130(.a(new_n210), .b(new_n225), .c(new_n128), .d(new_n212), .o1(new_n226));
  tech160nm_fixorc02aa1n03p5x5 g131(.a(\a[18] ), .b(\b[17] ), .out0(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n226), .c(new_n218), .out0(\s[18] ));
  norb02aa1n02x5               g133(.a(new_n227), .b(new_n213), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n225), .c(new_n128), .d(new_n212), .o1(new_n230));
  oaoi03aa1n02x5               g135(.a(\a[18] ), .b(\b[17] ), .c(new_n218), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[19] ), .b(\b[18] ), .out0(new_n233));
  xnbna2aa1n03x5               g138(.a(new_n233), .b(new_n230), .c(new_n232), .out0(\s[19] ));
  xnrc02aa1n02x5               g139(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g140(.a(new_n233), .b(new_n231), .c(new_n209), .d(new_n229), .o1(new_n236));
  nor042aa1n02x5               g141(.a(\b[19] ), .b(\a[20] ), .o1(new_n237));
  and002aa1n12x5               g142(.a(\b[19] ), .b(\a[20] ), .o(new_n238));
  nor042aa1n03x5               g143(.a(new_n238), .b(new_n237), .o1(new_n239));
  nor042aa1n02x5               g144(.a(\b[18] ), .b(\a[19] ), .o1(new_n240));
  oab012aa1n02x4               g145(.a(new_n240), .b(new_n238), .c(new_n237), .out0(new_n241));
  inv000aa1n02x5               g146(.a(new_n240), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n233), .o1(new_n243));
  aoai13aa1n02x7               g148(.a(new_n242), .b(new_n243), .c(new_n230), .d(new_n232), .o1(new_n244));
  aoi022aa1n03x5               g149(.a(new_n244), .b(new_n239), .c(new_n236), .d(new_n241), .o1(\s[20] ));
  nanp02aa1n03x5               g150(.a(new_n233), .b(new_n239), .o1(new_n246));
  nano22aa1n03x7               g151(.a(new_n246), .b(new_n210), .c(new_n227), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n225), .c(new_n128), .d(new_n212), .o1(new_n248));
  oai022aa1n12x5               g153(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n249));
  oaih22aa1n06x5               g154(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n250));
  aoi022aa1n06x5               g155(.a(\b[18] ), .b(\a[19] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(new_n250), .b(new_n251), .o1(new_n252));
  oab012aa1d15x5               g157(.a(new_n237), .b(new_n242), .c(new_n238), .out0(new_n253));
  oai013aa1n09x5               g158(.a(new_n253), .b(new_n252), .c(new_n238), .d(new_n249), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n248), .b(new_n255), .o1(new_n256));
  tech160nm_finand02aa1n05x5   g161(.a(\b[20] ), .b(\a[21] ), .o1(new_n257));
  nor002aa1n20x5               g162(.a(\b[20] ), .b(\a[21] ), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n257), .b(new_n258), .out0(new_n259));
  nona23aa1n09x5               g164(.a(new_n250), .b(new_n251), .c(new_n249), .d(new_n238), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n259), .o1(new_n261));
  and003aa1n02x5               g166(.a(new_n260), .b(new_n261), .c(new_n253), .o(new_n262));
  aoi022aa1n02x5               g167(.a(new_n256), .b(new_n259), .c(new_n248), .d(new_n262), .o1(\s[21] ));
  aoai13aa1n03x5               g168(.a(new_n259), .b(new_n254), .c(new_n209), .d(new_n247), .o1(new_n264));
  nor042aa1n06x5               g169(.a(\b[21] ), .b(\a[22] ), .o1(new_n265));
  nand42aa1n16x5               g170(.a(\b[21] ), .b(\a[22] ), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n266), .b(new_n265), .out0(new_n267));
  aoib12aa1n02x5               g172(.a(new_n258), .b(new_n266), .c(new_n265), .out0(new_n268));
  inv000aa1d42x5               g173(.a(new_n258), .o1(new_n269));
  aoai13aa1n04x5               g174(.a(new_n269), .b(new_n261), .c(new_n248), .d(new_n255), .o1(new_n270));
  aoi022aa1n02x7               g175(.a(new_n270), .b(new_n267), .c(new_n264), .d(new_n268), .o1(\s[22] ));
  nano23aa1n06x5               g176(.a(new_n265), .b(new_n258), .c(new_n266), .d(new_n257), .out0(new_n272));
  nano32aa1n02x4               g177(.a(new_n246), .b(new_n272), .c(new_n210), .d(new_n227), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n225), .c(new_n128), .d(new_n212), .o1(new_n274));
  oai022aa1n02x5               g179(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n275));
  aoi022aa1n03x5               g180(.a(new_n254), .b(new_n272), .c(new_n266), .d(new_n275), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(new_n274), .b(new_n276), .o1(new_n277));
  nor042aa1n09x5               g182(.a(\b[22] ), .b(\a[23] ), .o1(new_n278));
  tech160nm_finand02aa1n05x5   g183(.a(\b[22] ), .b(\a[23] ), .o1(new_n279));
  norb02aa1n12x5               g184(.a(new_n279), .b(new_n278), .out0(new_n280));
  aoi122aa1n02x5               g185(.a(new_n280), .b(new_n266), .c(new_n275), .d(new_n254), .e(new_n272), .o1(new_n281));
  aoi022aa1n02x5               g186(.a(new_n277), .b(new_n280), .c(new_n274), .d(new_n281), .o1(\s[23] ));
  inv040aa1n02x5               g187(.a(new_n276), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n280), .b(new_n283), .c(new_n209), .d(new_n273), .o1(new_n284));
  nor042aa1d18x5               g189(.a(\b[23] ), .b(\a[24] ), .o1(new_n285));
  nand02aa1n04x5               g190(.a(\b[23] ), .b(\a[24] ), .o1(new_n286));
  norb02aa1n06x4               g191(.a(new_n286), .b(new_n285), .out0(new_n287));
  inv000aa1d42x5               g192(.a(new_n285), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n278), .b(new_n288), .c(new_n286), .o1(new_n289));
  inv000aa1n04x5               g194(.a(new_n278), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n280), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n290), .b(new_n291), .c(new_n274), .d(new_n276), .o1(new_n292));
  aoi022aa1n03x5               g197(.a(new_n292), .b(new_n287), .c(new_n284), .d(new_n289), .o1(\s[24] ));
  nand23aa1n09x5               g198(.a(new_n272), .b(new_n280), .c(new_n287), .o1(new_n294));
  nano23aa1n02x4               g199(.a(new_n294), .b(new_n246), .c(new_n227), .d(new_n210), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n225), .c(new_n128), .d(new_n212), .o1(new_n296));
  nanb03aa1n03x5               g201(.a(new_n285), .b(new_n286), .c(new_n279), .out0(new_n297));
  oai112aa1n06x5               g202(.a(new_n290), .b(new_n266), .c(new_n265), .d(new_n258), .o1(new_n298));
  aob012aa1n09x5               g203(.a(new_n288), .b(new_n278), .c(new_n286), .out0(new_n299));
  oab012aa1n06x5               g204(.a(new_n299), .b(new_n298), .c(new_n297), .out0(new_n300));
  aoai13aa1n12x5               g205(.a(new_n300), .b(new_n294), .c(new_n260), .d(new_n253), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  nanp02aa1n02x5               g207(.a(new_n296), .b(new_n302), .o1(new_n303));
  xorc02aa1n12x5               g208(.a(\a[25] ), .b(\b[24] ), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n299), .b(new_n304), .o1(new_n305));
  oai012aa1n02x5               g210(.a(new_n305), .b(new_n298), .c(new_n297), .o1(new_n306));
  aoib12aa1n02x5               g211(.a(new_n306), .b(new_n254), .c(new_n294), .out0(new_n307));
  aoi022aa1n02x5               g212(.a(new_n303), .b(new_n304), .c(new_n296), .d(new_n307), .o1(\s[25] ));
  aoai13aa1n03x5               g213(.a(new_n304), .b(new_n301), .c(new_n209), .d(new_n295), .o1(new_n309));
  xorc02aa1n02x5               g214(.a(\a[26] ), .b(\b[25] ), .out0(new_n310));
  norp02aa1n02x5               g215(.a(\b[24] ), .b(\a[25] ), .o1(new_n311));
  norp02aa1n02x5               g216(.a(new_n310), .b(new_n311), .o1(new_n312));
  inv000aa1n03x5               g217(.a(new_n311), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n304), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n313), .b(new_n314), .c(new_n296), .d(new_n302), .o1(new_n315));
  aoi022aa1n02x7               g220(.a(new_n315), .b(new_n310), .c(new_n309), .d(new_n312), .o1(\s[26] ));
  and002aa1n06x5               g221(.a(new_n310), .b(new_n304), .o(new_n317));
  nano22aa1n03x7               g222(.a(new_n294), .b(new_n247), .c(new_n317), .out0(new_n318));
  aoai13aa1n12x5               g223(.a(new_n318), .b(new_n225), .c(new_n128), .d(new_n212), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[26] ), .b(\b[25] ), .c(new_n313), .carry(new_n320));
  inv000aa1d42x5               g225(.a(new_n320), .o1(new_n321));
  aoi012aa1n09x5               g226(.a(new_n321), .b(new_n301), .c(new_n317), .o1(new_n322));
  nanp02aa1n02x5               g227(.a(new_n319), .b(new_n322), .o1(new_n323));
  xorc02aa1n12x5               g228(.a(\a[27] ), .b(\b[26] ), .out0(new_n324));
  aoi112aa1n02x5               g229(.a(new_n324), .b(new_n321), .c(new_n301), .d(new_n317), .o1(new_n325));
  aoi022aa1n02x5               g230(.a(new_n323), .b(new_n324), .c(new_n319), .d(new_n325), .o1(\s[27] ));
  aob012aa1n06x5               g231(.a(new_n320), .b(new_n301), .c(new_n317), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n324), .b(new_n327), .c(new_n209), .d(new_n318), .o1(new_n328));
  xorc02aa1n12x5               g233(.a(\a[28] ), .b(\b[27] ), .out0(new_n329));
  norp02aa1n02x5               g234(.a(\b[26] ), .b(\a[27] ), .o1(new_n330));
  norp02aa1n02x5               g235(.a(new_n329), .b(new_n330), .o1(new_n331));
  inv000aa1n03x5               g236(.a(new_n330), .o1(new_n332));
  inv000aa1n06x5               g237(.a(new_n324), .o1(new_n333));
  aoai13aa1n03x5               g238(.a(new_n332), .b(new_n333), .c(new_n319), .d(new_n322), .o1(new_n334));
  aoi022aa1n03x5               g239(.a(new_n334), .b(new_n329), .c(new_n328), .d(new_n331), .o1(\s[28] ));
  and002aa1n02x5               g240(.a(new_n329), .b(new_n324), .o(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n327), .c(new_n209), .d(new_n318), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n336), .o1(new_n338));
  oao003aa1n02x5               g243(.a(\a[28] ), .b(\b[27] ), .c(new_n332), .carry(new_n339));
  aoai13aa1n03x5               g244(.a(new_n339), .b(new_n338), .c(new_n319), .d(new_n322), .o1(new_n340));
  xorc02aa1n12x5               g245(.a(\a[29] ), .b(\b[28] ), .out0(new_n341));
  norb02aa1n02x5               g246(.a(new_n339), .b(new_n341), .out0(new_n342));
  aoi022aa1n03x5               g247(.a(new_n340), .b(new_n341), .c(new_n337), .d(new_n342), .o1(\s[29] ));
  xorb03aa1n02x5               g248(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g249(.a(new_n333), .b(new_n329), .c(new_n341), .out0(new_n345));
  aoai13aa1n02x5               g250(.a(new_n345), .b(new_n327), .c(new_n209), .d(new_n318), .o1(new_n346));
  inv000aa1n02x5               g251(.a(new_n345), .o1(new_n347));
  inv000aa1d42x5               g252(.a(\b[28] ), .o1(new_n348));
  inv000aa1d42x5               g253(.a(\a[29] ), .o1(new_n349));
  oaib12aa1n02x5               g254(.a(new_n339), .b(\b[28] ), .c(new_n349), .out0(new_n350));
  oaib12aa1n02x5               g255(.a(new_n350), .b(new_n348), .c(\a[29] ), .out0(new_n351));
  aoai13aa1n03x5               g256(.a(new_n351), .b(new_n347), .c(new_n319), .d(new_n322), .o1(new_n352));
  xorc02aa1n02x5               g257(.a(\a[30] ), .b(\b[29] ), .out0(new_n353));
  oaoi13aa1n02x5               g258(.a(new_n353), .b(new_n350), .c(new_n349), .d(new_n348), .o1(new_n354));
  aoi022aa1n03x5               g259(.a(new_n352), .b(new_n353), .c(new_n346), .d(new_n354), .o1(\s[30] ));
  nano32aa1n02x4               g260(.a(new_n333), .b(new_n353), .c(new_n329), .d(new_n341), .out0(new_n356));
  aoai13aa1n03x5               g261(.a(new_n356), .b(new_n327), .c(new_n209), .d(new_n318), .o1(new_n357));
  aoi022aa1n02x5               g262(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n358));
  norb02aa1n02x5               g263(.a(\b[30] ), .b(\a[31] ), .out0(new_n359));
  obai22aa1n02x7               g264(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n360));
  aoi112aa1n02x5               g265(.a(new_n360), .b(new_n359), .c(new_n350), .d(new_n358), .o1(new_n361));
  xorc02aa1n02x5               g266(.a(\a[31] ), .b(\b[30] ), .out0(new_n362));
  inv000aa1n02x5               g267(.a(new_n356), .o1(new_n363));
  norp02aa1n02x5               g268(.a(\b[29] ), .b(\a[30] ), .o1(new_n364));
  aoi012aa1n02x5               g269(.a(new_n364), .b(new_n350), .c(new_n358), .o1(new_n365));
  aoai13aa1n03x5               g270(.a(new_n365), .b(new_n363), .c(new_n319), .d(new_n322), .o1(new_n366));
  aoi022aa1n02x7               g271(.a(new_n366), .b(new_n362), .c(new_n357), .d(new_n361), .o1(\s[31] ));
  aoi112aa1n02x5               g272(.a(new_n108), .b(new_n98), .c(new_n100), .d(new_n101), .o1(new_n368));
  aoi012aa1n02x5               g273(.a(new_n368), .b(new_n102), .c(new_n108), .o1(\s[3] ));
  oaoi03aa1n02x5               g274(.a(\a[3] ), .b(\b[2] ), .c(new_n132), .o1(new_n370));
  aoi112aa1n02x5               g275(.a(new_n106), .b(new_n105), .c(new_n102), .d(new_n107), .o1(new_n371));
  aoi012aa1n02x5               g276(.a(new_n371), .b(new_n105), .c(new_n370), .o1(\s[4] ));
  norb02aa1n02x5               g277(.a(new_n113), .b(new_n114), .out0(new_n373));
  xnbna2aa1n03x5               g278(.a(new_n373), .b(new_n109), .c(new_n110), .out0(\s[5] ));
  norb02aa1n02x5               g279(.a(new_n111), .b(new_n112), .out0(new_n375));
  aoai13aa1n02x5               g280(.a(new_n375), .b(new_n114), .c(new_n134), .d(new_n113), .o1(new_n376));
  aoi112aa1n02x5               g281(.a(new_n114), .b(new_n375), .c(new_n134), .d(new_n373), .o1(new_n377));
  norb02aa1n02x5               g282(.a(new_n376), .b(new_n377), .out0(\s[6] ));
  norb02aa1n02x5               g283(.a(new_n118), .b(new_n119), .out0(new_n379));
  oai012aa1n02x5               g284(.a(new_n111), .b(new_n114), .c(new_n112), .o1(new_n380));
  xnbna2aa1n03x5               g285(.a(new_n379), .b(new_n376), .c(new_n380), .out0(\s[7] ));
  aob012aa1n02x5               g286(.a(new_n379), .b(new_n376), .c(new_n380), .out0(new_n382));
  nanp02aa1n02x5               g287(.a(new_n382), .b(new_n123), .o1(new_n383));
  aoib12aa1n02x5               g288(.a(new_n119), .b(new_n117), .c(new_n116), .out0(new_n384));
  aoi022aa1n02x5               g289(.a(new_n383), .b(new_n122), .c(new_n382), .d(new_n384), .o1(\s[8] ));
  aoi113aa1n02x5               g290(.a(new_n130), .b(new_n124), .c(new_n125), .d(new_n126), .e(new_n122), .o1(new_n386));
  aoi022aa1n02x5               g291(.a(new_n128), .b(new_n130), .c(new_n135), .d(new_n386), .o1(\s[9] ));
endmodule


