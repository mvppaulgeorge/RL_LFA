// Benchmark "adder" written by ABC on Thu Jul 18 09:25:51 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n174, new_n175, new_n176,
    new_n177, new_n178, new_n179, new_n180, new_n181, new_n183, new_n184,
    new_n185, new_n186, new_n187, new_n189, new_n190, new_n191, new_n192,
    new_n193, new_n194, new_n195, new_n196, new_n198, new_n199, new_n200,
    new_n201, new_n202, new_n203, new_n204, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n223,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n229, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n241, new_n242, new_n243, new_n244, new_n245, new_n246, new_n247,
    new_n248, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n267, new_n268, new_n269, new_n270,
    new_n271, new_n273, new_n274, new_n275, new_n276, new_n277, new_n278,
    new_n279, new_n280, new_n281, new_n282, new_n283, new_n284, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n306, new_n307, new_n308, new_n309,
    new_n310, new_n312, new_n313, new_n314, new_n315, new_n316, new_n317,
    new_n318, new_n319, new_n320, new_n321, new_n322, new_n324, new_n325,
    new_n326, new_n327, new_n328, new_n329, new_n330, new_n331, new_n332,
    new_n334, new_n335, new_n336, new_n337, new_n338, new_n339, new_n340,
    new_n343, new_n344, new_n345, new_n346, new_n347, new_n348, new_n349,
    new_n350, new_n351, new_n353, new_n354, new_n355, new_n356, new_n357,
    new_n358, new_n359, new_n360, new_n361, new_n362, new_n363, new_n366,
    new_n368, new_n370, new_n372, new_n373, new_n374, new_n375, new_n378;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n16x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n12x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor042aa1d18x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor002aa1n20x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi022aa1n06x5               g006(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n102));
  xorc02aa1n12x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  nor042aa1d18x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand02aa1d10x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n03x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  oai112aa1n06x5               g011(.a(new_n103), .b(new_n106), .c(new_n102), .d(new_n101), .o1(new_n107));
  inv040aa1d32x5               g012(.a(\a[4] ), .o1(new_n108));
  inv040aa1d28x5               g013(.a(\b[3] ), .o1(new_n109));
  oaoi03aa1n12x5               g014(.a(new_n108), .b(new_n109), .c(new_n104), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand02aa1d28x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanb02aa1d36x5               g017(.a(new_n111), .b(new_n112), .out0(new_n113));
  inv000aa1d42x5               g018(.a(\b[4] ), .o1(new_n114));
  nanb02aa1d36x5               g019(.a(\a[5] ), .b(new_n114), .out0(new_n115));
  nanp02aa1n12x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nand22aa1n12x5               g021(.a(new_n115), .b(new_n116), .o1(new_n117));
  nand02aa1d16x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor002aa1d32x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nor002aa1n03x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nand42aa1n20x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  nano23aa1n02x4               g026(.a(new_n120), .b(new_n119), .c(new_n121), .d(new_n118), .out0(new_n122));
  nona22aa1n03x5               g027(.a(new_n122), .b(new_n113), .c(new_n117), .out0(new_n123));
  nano22aa1n03x7               g028(.a(new_n111), .b(new_n112), .c(new_n121), .out0(new_n124));
  norp02aa1n06x5               g029(.a(\b[4] ), .b(\a[5] ), .o1(new_n125));
  tech160nm_fioai012aa1n03p5x5 g030(.a(new_n118), .b(\b[7] ), .c(\a[8] ), .o1(new_n126));
  oab012aa1n06x5               g031(.a(new_n126), .b(new_n125), .c(new_n119), .out0(new_n127));
  inv040aa1n02x5               g032(.a(new_n111), .o1(new_n128));
  oaoi03aa1n09x5               g033(.a(\a[8] ), .b(\b[7] ), .c(new_n128), .o1(new_n129));
  aoi012aa1d18x5               g034(.a(new_n129), .b(new_n127), .c(new_n124), .o1(new_n130));
  aoai13aa1n12x5               g035(.a(new_n130), .b(new_n123), .c(new_n107), .d(new_n110), .o1(new_n131));
  xorc02aa1n12x5               g036(.a(\a[9] ), .b(\b[8] ), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n99), .b(new_n100), .c(new_n131), .d(new_n132), .o1(new_n133));
  norb03aa1n02x5               g038(.a(new_n98), .b(new_n97), .c(new_n100), .out0(new_n134));
  aob012aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n132), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(new_n133), .b(new_n135), .o1(\s[10] ));
  nand02aa1d12x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nor002aa1d32x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  inv000aa1n12x5               g043(.a(new_n138), .o1(new_n139));
  aoi022aa1n02x5               g044(.a(new_n135), .b(new_n98), .c(new_n137), .d(new_n139), .o1(new_n140));
  tech160nm_fiaoi012aa1n04x5   g045(.a(new_n138), .b(\a[10] ), .c(\b[9] ), .o1(new_n141));
  aoi013aa1n02x4               g046(.a(new_n140), .b(new_n137), .c(new_n135), .d(new_n141), .o1(\s[11] ));
  nanp03aa1n03x5               g047(.a(new_n135), .b(new_n137), .c(new_n141), .o1(new_n143));
  nor002aa1d32x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1d16x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n03x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n143), .c(new_n139), .out0(\s[12] ));
  nona23aa1n03x5               g052(.a(new_n145), .b(new_n137), .c(new_n138), .d(new_n144), .out0(new_n148));
  norb03aa1n03x5               g053(.a(new_n132), .b(new_n148), .c(new_n99), .out0(new_n149));
  nanp02aa1n02x5               g054(.a(new_n131), .b(new_n149), .o1(new_n150));
  nand42aa1n08x5               g055(.a(\b[1] ), .b(\a[2] ), .o1(new_n151));
  nand42aa1n10x5               g056(.a(\b[0] ), .b(\a[1] ), .o1(new_n152));
  aoi012aa1n12x5               g057(.a(new_n101), .b(new_n151), .c(new_n152), .o1(new_n153));
  nand02aa1n04x5               g058(.a(\b[3] ), .b(\a[4] ), .o1(new_n154));
  nand42aa1n03x5               g059(.a(new_n109), .b(new_n108), .o1(new_n155));
  nanp02aa1n06x5               g060(.a(new_n155), .b(new_n154), .o1(new_n156));
  inv040aa1d32x5               g061(.a(\a[3] ), .o1(new_n157));
  inv040aa1n18x5               g062(.a(\b[2] ), .o1(new_n158));
  nanp02aa1n06x5               g063(.a(new_n158), .b(new_n157), .o1(new_n159));
  nand02aa1n02x5               g064(.a(new_n159), .b(new_n105), .o1(new_n160));
  nor003aa1n03x5               g065(.a(new_n153), .b(new_n156), .c(new_n160), .o1(new_n161));
  inv000aa1n02x5               g066(.a(new_n110), .o1(new_n162));
  nona23aa1n09x5               g067(.a(new_n116), .b(new_n112), .c(new_n111), .d(new_n125), .out0(new_n163));
  norb02aa1n06x5               g068(.a(new_n118), .b(new_n119), .out0(new_n164));
  norb02aa1n03x5               g069(.a(new_n121), .b(new_n120), .out0(new_n165));
  nano22aa1n03x7               g070(.a(new_n163), .b(new_n164), .c(new_n165), .out0(new_n166));
  oai012aa1n09x5               g071(.a(new_n166), .b(new_n161), .c(new_n162), .o1(new_n167));
  nanb02aa1n02x5               g072(.a(new_n138), .b(new_n137), .out0(new_n168));
  nona23aa1n03x5               g073(.a(new_n146), .b(new_n132), .c(new_n99), .d(new_n168), .out0(new_n169));
  nanb03aa1n12x5               g074(.a(new_n144), .b(new_n145), .c(new_n137), .out0(new_n170));
  oai112aa1n06x5               g075(.a(new_n139), .b(new_n98), .c(new_n100), .d(new_n97), .o1(new_n171));
  aoi012aa1n12x5               g076(.a(new_n144), .b(new_n138), .c(new_n145), .o1(new_n172));
  oai012aa1n18x5               g077(.a(new_n172), .b(new_n171), .c(new_n170), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n169), .c(new_n167), .d(new_n130), .o1(new_n175));
  nor002aa1d32x5               g080(.a(\b[12] ), .b(\a[13] ), .o1(new_n176));
  nand02aa1n06x5               g081(.a(\b[12] ), .b(\a[13] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  nano22aa1n02x4               g083(.a(new_n144), .b(new_n137), .c(new_n145), .out0(new_n179));
  oai112aa1n02x5               g084(.a(new_n179), .b(new_n141), .c(new_n100), .d(new_n97), .o1(new_n180));
  nano22aa1n02x4               g085(.a(new_n178), .b(new_n180), .c(new_n172), .out0(new_n181));
  aoi022aa1n02x5               g086(.a(new_n175), .b(new_n178), .c(new_n150), .d(new_n181), .o1(\s[13] ));
  nor002aa1d24x5               g087(.a(\b[13] ), .b(\a[14] ), .o1(new_n183));
  nand02aa1n08x5               g088(.a(\b[13] ), .b(\a[14] ), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n176), .c(new_n175), .d(new_n177), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(new_n176), .b(new_n185), .c(new_n175), .d(new_n178), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(\s[14] ));
  oa0012aa1n02x5               g093(.a(new_n184), .b(new_n183), .c(new_n176), .o(new_n189));
  nano23aa1n06x5               g094(.a(new_n176), .b(new_n183), .c(new_n184), .d(new_n177), .out0(new_n190));
  nor002aa1d32x5               g095(.a(\b[14] ), .b(\a[15] ), .o1(new_n191));
  nand02aa1d16x5               g096(.a(\b[14] ), .b(\a[15] ), .o1(new_n192));
  nanb02aa1d36x5               g097(.a(new_n191), .b(new_n192), .out0(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n189), .c(new_n175), .d(new_n190), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n189), .b(new_n194), .c(new_n175), .d(new_n190), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(\s[15] ));
  inv040aa1d32x5               g102(.a(\a[16] ), .o1(new_n198));
  inv030aa1d32x5               g103(.a(\b[15] ), .o1(new_n199));
  nand02aa1n04x5               g104(.a(new_n199), .b(new_n198), .o1(new_n200));
  nand02aa1d06x5               g105(.a(\b[15] ), .b(\a[16] ), .o1(new_n201));
  nanp02aa1n04x5               g106(.a(new_n200), .b(new_n201), .o1(new_n202));
  oaoi13aa1n06x5               g107(.a(new_n202), .b(new_n195), .c(\a[15] ), .d(\b[14] ), .o1(new_n203));
  aoi012aa1n02x5               g108(.a(new_n191), .b(new_n200), .c(new_n201), .o1(new_n204));
  aoi012aa1n02x5               g109(.a(new_n203), .b(new_n195), .c(new_n204), .o1(\s[16] ));
  nona22aa1n03x5               g110(.a(new_n190), .b(new_n193), .c(new_n202), .out0(new_n206));
  nor042aa1n03x5               g111(.a(new_n169), .b(new_n206), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(new_n131), .b(new_n207), .o1(new_n208));
  nona23aa1n09x5               g113(.a(new_n184), .b(new_n177), .c(new_n176), .d(new_n183), .out0(new_n209));
  nor043aa1n03x5               g114(.a(new_n209), .b(new_n193), .c(new_n202), .o1(new_n210));
  nand02aa1n02x5               g115(.a(new_n149), .b(new_n210), .o1(new_n211));
  nanp03aa1n02x5               g116(.a(new_n200), .b(new_n192), .c(new_n201), .o1(new_n212));
  oai122aa1n06x5               g117(.a(new_n184), .b(new_n183), .c(new_n176), .d(\b[14] ), .e(\a[15] ), .o1(new_n213));
  oaoi03aa1n02x5               g118(.a(new_n198), .b(new_n199), .c(new_n191), .o1(new_n214));
  oaih12aa1n02x5               g119(.a(new_n214), .b(new_n213), .c(new_n212), .o1(new_n215));
  aoi012aa1n12x5               g120(.a(new_n215), .b(new_n210), .c(new_n173), .o1(new_n216));
  aoai13aa1n12x5               g121(.a(new_n216), .b(new_n211), .c(new_n167), .d(new_n130), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[17] ), .b(\b[16] ), .out0(new_n218));
  norp02aa1n02x5               g123(.a(new_n213), .b(new_n212), .o1(new_n219));
  nanb02aa1n02x5               g124(.a(new_n218), .b(new_n214), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n219), .b(new_n220), .c(new_n210), .d(new_n173), .o1(new_n221));
  aoi022aa1n02x5               g126(.a(new_n217), .b(new_n218), .c(new_n208), .d(new_n221), .o1(\s[17] ));
  inv000aa1d42x5               g127(.a(\a[17] ), .o1(new_n223));
  nanb02aa1n02x5               g128(.a(\b[16] ), .b(new_n223), .out0(new_n224));
  inv000aa1n02x5               g129(.a(new_n214), .o1(new_n225));
  oab012aa1n02x5               g130(.a(new_n225), .b(new_n213), .c(new_n212), .out0(new_n226));
  aoai13aa1n04x5               g131(.a(new_n226), .b(new_n206), .c(new_n180), .d(new_n172), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n218), .b(new_n227), .c(new_n131), .d(new_n207), .o1(new_n228));
  xorc02aa1n02x5               g133(.a(\a[18] ), .b(\b[17] ), .out0(new_n229));
  xnbna2aa1n03x5               g134(.a(new_n229), .b(new_n228), .c(new_n224), .out0(\s[18] ));
  nor002aa1d32x5               g135(.a(\b[16] ), .b(\a[17] ), .o1(new_n231));
  nor002aa1n12x5               g136(.a(\b[17] ), .b(\a[18] ), .o1(new_n232));
  nand02aa1d28x5               g137(.a(\b[17] ), .b(\a[18] ), .o1(new_n233));
  oai012aa1n02x5               g138(.a(new_n233), .b(new_n232), .c(new_n231), .o1(new_n234));
  nand02aa1n06x5               g139(.a(\b[16] ), .b(\a[17] ), .o1(new_n235));
  nano23aa1n06x5               g140(.a(new_n231), .b(new_n232), .c(new_n233), .d(new_n235), .out0(new_n236));
  aoai13aa1n04x5               g141(.a(new_n236), .b(new_n227), .c(new_n131), .d(new_n207), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[19] ), .b(\b[18] ), .out0(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n237), .c(new_n234), .out0(\s[19] ));
  xnrc02aa1n02x5               g144(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n03x5               g145(.a(new_n238), .b(new_n237), .c(new_n234), .out0(new_n241));
  inv000aa1d42x5               g146(.a(\a[19] ), .o1(new_n242));
  oaib12aa1n06x5               g147(.a(new_n241), .b(\b[18] ), .c(new_n242), .out0(new_n243));
  nor042aa1n06x5               g148(.a(\b[19] ), .b(\a[20] ), .o1(new_n244));
  nand42aa1d28x5               g149(.a(\b[19] ), .b(\a[20] ), .o1(new_n245));
  norb02aa1n06x4               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  inv000aa1d42x5               g151(.a(\b[18] ), .o1(new_n247));
  aboi22aa1n03x5               g152(.a(new_n244), .b(new_n245), .c(new_n242), .d(new_n247), .out0(new_n248));
  aoi022aa1n02x5               g153(.a(new_n243), .b(new_n246), .c(new_n241), .d(new_n248), .o1(\s[20] ));
  nand23aa1n06x5               g154(.a(new_n236), .b(new_n238), .c(new_n246), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  nand42aa1d28x5               g156(.a(\b[18] ), .b(\a[19] ), .o1(new_n252));
  nanb03aa1n02x5               g157(.a(new_n244), .b(new_n245), .c(new_n252), .out0(new_n253));
  oai022aa1n04x7               g158(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n254));
  oai112aa1n02x5               g159(.a(new_n254), .b(new_n233), .c(\b[18] ), .d(\a[19] ), .o1(new_n255));
  aoi013aa1n06x4               g160(.a(new_n244), .b(new_n245), .c(new_n242), .d(new_n247), .o1(new_n256));
  oai012aa1n06x5               g161(.a(new_n256), .b(new_n255), .c(new_n253), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[21] ), .b(\b[20] ), .out0(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n257), .c(new_n217), .d(new_n251), .o1(new_n259));
  tech160nm_finor002aa1n03p5x5 g164(.a(new_n232), .b(new_n231), .o1(new_n260));
  nano22aa1n03x7               g165(.a(new_n244), .b(new_n252), .c(new_n245), .out0(new_n261));
  oai012aa1n02x5               g166(.a(new_n233), .b(\b[18] ), .c(\a[19] ), .o1(new_n262));
  nona22aa1n09x5               g167(.a(new_n261), .b(new_n260), .c(new_n262), .out0(new_n263));
  nano22aa1n02x4               g168(.a(new_n258), .b(new_n263), .c(new_n256), .out0(new_n264));
  aobi12aa1n02x5               g169(.a(new_n264), .b(new_n217), .c(new_n251), .out0(new_n265));
  norb02aa1n02x5               g170(.a(new_n259), .b(new_n265), .out0(\s[21] ));
  inv000aa1d42x5               g171(.a(\a[21] ), .o1(new_n267));
  oaib12aa1n03x5               g172(.a(new_n259), .b(\b[20] ), .c(new_n267), .out0(new_n268));
  tech160nm_fixorc02aa1n05x5   g173(.a(\a[22] ), .b(\b[21] ), .out0(new_n269));
  nor022aa1n16x5               g174(.a(\b[20] ), .b(\a[21] ), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n269), .b(new_n270), .o1(new_n271));
  aoi022aa1n02x7               g176(.a(new_n268), .b(new_n269), .c(new_n259), .d(new_n271), .o1(\s[22] ));
  nanp02aa1n03x5               g177(.a(new_n269), .b(new_n258), .o1(new_n273));
  nona22aa1n06x5               g178(.a(new_n217), .b(new_n250), .c(new_n273), .out0(new_n274));
  inv040aa1d32x5               g179(.a(\a[22] ), .o1(new_n275));
  inv040aa1d32x5               g180(.a(\b[21] ), .o1(new_n276));
  oaoi03aa1n12x5               g181(.a(new_n275), .b(new_n276), .c(new_n270), .o1(new_n277));
  aoai13aa1n12x5               g182(.a(new_n277), .b(new_n273), .c(new_n263), .d(new_n256), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[23] ), .b(\b[22] ), .out0(new_n280));
  aob012aa1n03x5               g185(.a(new_n280), .b(new_n274), .c(new_n279), .out0(new_n281));
  xroi22aa1d04x5               g186(.a(new_n267), .b(\b[20] ), .c(new_n275), .d(\b[21] ), .out0(new_n282));
  inv000aa1d42x5               g187(.a(new_n277), .o1(new_n283));
  aoi112aa1n02x5               g188(.a(new_n280), .b(new_n283), .c(new_n257), .d(new_n282), .o1(new_n284));
  aobi12aa1n02x7               g189(.a(new_n281), .b(new_n284), .c(new_n274), .out0(\s[23] ));
  nor002aa1d24x5               g190(.a(\b[22] ), .b(\a[23] ), .o1(new_n286));
  inv040aa1n03x5               g191(.a(new_n286), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n280), .o1(new_n288));
  aoai13aa1n02x5               g193(.a(new_n287), .b(new_n288), .c(new_n274), .d(new_n279), .o1(new_n289));
  xorc02aa1n02x5               g194(.a(\a[24] ), .b(\b[23] ), .out0(new_n290));
  norp02aa1n02x5               g195(.a(new_n290), .b(new_n286), .o1(new_n291));
  aoi022aa1n03x5               g196(.a(new_n289), .b(new_n290), .c(new_n281), .d(new_n291), .o1(\s[24] ));
  nand42aa1n02x5               g197(.a(\b[22] ), .b(\a[23] ), .o1(new_n293));
  xnrc02aa1n03x5               g198(.a(\b[23] ), .b(\a[24] ), .out0(new_n294));
  nano22aa1n03x7               g199(.a(new_n294), .b(new_n287), .c(new_n293), .out0(new_n295));
  nano22aa1n02x5               g200(.a(new_n250), .b(new_n282), .c(new_n295), .out0(new_n296));
  aobi12aa1n02x5               g201(.a(new_n296), .b(new_n208), .c(new_n216), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n295), .b(new_n283), .c(new_n257), .d(new_n282), .o1(new_n298));
  oaoi03aa1n09x5               g203(.a(\a[24] ), .b(\b[23] ), .c(new_n287), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n299), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[25] ), .b(\b[24] ), .out0(new_n302));
  aoai13aa1n06x5               g207(.a(new_n302), .b(new_n301), .c(new_n217), .d(new_n296), .o1(new_n303));
  nona22aa1n02x4               g208(.a(new_n298), .b(new_n299), .c(new_n302), .out0(new_n304));
  oa0012aa1n03x5               g209(.a(new_n303), .b(new_n304), .c(new_n297), .o(\s[25] ));
  inv000aa1d42x5               g210(.a(\a[25] ), .o1(new_n306));
  oaib12aa1n03x5               g211(.a(new_n303), .b(\b[24] ), .c(new_n306), .out0(new_n307));
  xorc02aa1n02x5               g212(.a(\a[26] ), .b(\b[25] ), .out0(new_n308));
  norp02aa1n02x5               g213(.a(\b[24] ), .b(\a[25] ), .o1(new_n309));
  norp02aa1n02x5               g214(.a(new_n308), .b(new_n309), .o1(new_n310));
  aoi022aa1n02x7               g215(.a(new_n307), .b(new_n308), .c(new_n303), .d(new_n310), .o1(\s[26] ));
  inv000aa1d42x5               g216(.a(\a[26] ), .o1(new_n312));
  xroi22aa1d04x5               g217(.a(new_n306), .b(\b[24] ), .c(new_n312), .d(\b[25] ), .out0(new_n313));
  aoai13aa1n09x5               g218(.a(new_n313), .b(new_n299), .c(new_n278), .d(new_n295), .o1(new_n314));
  nano32aa1n03x7               g219(.a(new_n250), .b(new_n313), .c(new_n282), .d(new_n295), .out0(new_n315));
  aoai13aa1n06x5               g220(.a(new_n315), .b(new_n227), .c(new_n131), .d(new_n207), .o1(new_n316));
  inv000aa1d42x5               g221(.a(\b[25] ), .o1(new_n317));
  oaoi03aa1n12x5               g222(.a(new_n312), .b(new_n317), .c(new_n309), .o1(new_n318));
  nand23aa1n06x5               g223(.a(new_n316), .b(new_n314), .c(new_n318), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[27] ), .b(\b[26] ), .out0(new_n320));
  inv000aa1d42x5               g225(.a(new_n318), .o1(new_n321));
  aoi112aa1n02x5               g226(.a(new_n320), .b(new_n321), .c(new_n217), .d(new_n315), .o1(new_n322));
  aoi022aa1n02x7               g227(.a(new_n322), .b(new_n314), .c(new_n319), .d(new_n320), .o1(\s[27] ));
  tech160nm_finand02aa1n05x5   g228(.a(new_n319), .b(new_n320), .o1(new_n324));
  aobi12aa1n06x5               g229(.a(new_n313), .b(new_n298), .c(new_n300), .out0(new_n325));
  aoi112aa1n06x5               g230(.a(new_n325), .b(new_n321), .c(new_n217), .d(new_n315), .o1(new_n326));
  oaoi03aa1n02x5               g231(.a(\a[27] ), .b(\b[26] ), .c(new_n326), .o1(new_n327));
  norp02aa1n02x5               g232(.a(\b[27] ), .b(\a[28] ), .o1(new_n328));
  nand42aa1n03x5               g233(.a(\b[27] ), .b(\a[28] ), .o1(new_n329));
  norb02aa1n02x5               g234(.a(new_n329), .b(new_n328), .out0(new_n330));
  norp02aa1n02x5               g235(.a(\b[26] ), .b(\a[27] ), .o1(new_n331));
  aoib12aa1n02x5               g236(.a(new_n331), .b(new_n329), .c(new_n328), .out0(new_n332));
  aoi022aa1n03x5               g237(.a(new_n327), .b(new_n330), .c(new_n324), .d(new_n332), .o1(\s[28] ));
  and002aa1n06x5               g238(.a(new_n320), .b(new_n330), .o(new_n334));
  tech160nm_finand02aa1n05x5   g239(.a(new_n319), .b(new_n334), .o1(new_n335));
  inv040aa1n03x5               g240(.a(new_n334), .o1(new_n336));
  oai012aa1n02x5               g241(.a(new_n329), .b(new_n328), .c(new_n331), .o1(new_n337));
  oai012aa1n03x5               g242(.a(new_n337), .b(new_n326), .c(new_n336), .o1(new_n338));
  xorc02aa1n02x5               g243(.a(\a[29] ), .b(\b[28] ), .out0(new_n339));
  norb02aa1n02x5               g244(.a(new_n337), .b(new_n339), .out0(new_n340));
  aoi022aa1n03x5               g245(.a(new_n338), .b(new_n339), .c(new_n335), .d(new_n340), .o1(\s[29] ));
  xorb03aa1n02x5               g246(.a(new_n152), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanp03aa1n02x5               g247(.a(new_n339), .b(new_n320), .c(new_n330), .o1(new_n343));
  nanb02aa1n03x5               g248(.a(new_n343), .b(new_n319), .out0(new_n344));
  inv000aa1d42x5               g249(.a(\b[28] ), .o1(new_n345));
  inv000aa1d42x5               g250(.a(\a[29] ), .o1(new_n346));
  oaib12aa1n02x5               g251(.a(new_n337), .b(\b[28] ), .c(new_n346), .out0(new_n347));
  oaib12aa1n02x5               g252(.a(new_n347), .b(new_n345), .c(\a[29] ), .out0(new_n348));
  oai012aa1n02x5               g253(.a(new_n348), .b(new_n326), .c(new_n343), .o1(new_n349));
  xorc02aa1n02x5               g254(.a(\a[30] ), .b(\b[29] ), .out0(new_n350));
  oaoi13aa1n02x5               g255(.a(new_n350), .b(new_n347), .c(new_n346), .d(new_n345), .o1(new_n351));
  aoi022aa1n03x5               g256(.a(new_n349), .b(new_n350), .c(new_n344), .d(new_n351), .o1(\s[30] ));
  nano22aa1n03x7               g257(.a(new_n336), .b(new_n339), .c(new_n350), .out0(new_n353));
  inv000aa1n02x5               g258(.a(new_n353), .o1(new_n354));
  aoi013aa1n02x5               g259(.a(new_n354), .b(new_n316), .c(new_n314), .d(new_n318), .o1(new_n355));
  aoi022aa1n02x5               g260(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n356));
  nanp02aa1n02x5               g261(.a(new_n347), .b(new_n356), .o1(new_n357));
  norb02aa1n02x5               g262(.a(\b[30] ), .b(\a[31] ), .out0(new_n358));
  obai22aa1n02x7               g263(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n359));
  nona22aa1n02x4               g264(.a(new_n357), .b(new_n358), .c(new_n359), .out0(new_n360));
  tech160nm_fiaoi012aa1n05x5   g265(.a(new_n360), .b(new_n319), .c(new_n353), .o1(new_n361));
  oai012aa1n02x5               g266(.a(new_n357), .b(\b[29] ), .c(\a[30] ), .o1(new_n362));
  xorc02aa1n02x5               g267(.a(\a[31] ), .b(\b[30] ), .out0(new_n363));
  oaoi13aa1n02x7               g268(.a(new_n361), .b(new_n363), .c(new_n362), .d(new_n355), .o1(\s[31] ));
  xnbna2aa1n03x5               g269(.a(new_n153), .b(new_n105), .c(new_n159), .out0(\s[3] ));
  oai012aa1n02x5               g270(.a(new_n106), .b(new_n102), .c(new_n101), .o1(new_n366));
  xnbna2aa1n03x5               g271(.a(new_n103), .b(new_n366), .c(new_n159), .out0(\s[4] ));
  inv000aa1d42x5               g272(.a(new_n117), .o1(new_n368));
  xnbna2aa1n03x5               g273(.a(new_n368), .b(new_n107), .c(new_n110), .out0(\s[5] ));
  oai012aa1n02x5               g274(.a(new_n368), .b(new_n161), .c(new_n162), .o1(new_n370));
  xnbna2aa1n03x5               g275(.a(new_n164), .b(new_n370), .c(new_n115), .out0(\s[6] ));
  inv000aa1d42x5               g276(.a(new_n113), .o1(new_n372));
  aoai13aa1n02x5               g277(.a(new_n115), .b(new_n117), .c(new_n107), .d(new_n110), .o1(new_n373));
  aoai13aa1n02x5               g278(.a(new_n372), .b(new_n119), .c(new_n373), .d(new_n118), .o1(new_n374));
  aoi112aa1n02x5               g279(.a(new_n372), .b(new_n119), .c(new_n373), .d(new_n164), .o1(new_n375));
  norb02aa1n02x5               g280(.a(new_n374), .b(new_n375), .out0(\s[7] ));
  xnbna2aa1n03x5               g281(.a(new_n165), .b(new_n374), .c(new_n128), .out0(\s[8] ));
  aoi112aa1n02x5               g282(.a(new_n129), .b(new_n132), .c(new_n127), .d(new_n124), .o1(new_n378));
  aoi022aa1n02x5               g283(.a(new_n131), .b(new_n132), .c(new_n167), .d(new_n378), .o1(\s[9] ));
endmodule


