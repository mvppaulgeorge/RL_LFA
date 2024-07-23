// Benchmark "adder" written by ABC on Wed Jul 17 15:31:21 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n300, new_n301, new_n302,
    new_n303, new_n304, new_n305, new_n306, new_n307, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n316, new_n317,
    new_n318, new_n319, new_n320, new_n322, new_n323, new_n324, new_n325,
    new_n326, new_n327, new_n328, new_n329, new_n331, new_n332, new_n333,
    new_n334, new_n335, new_n336, new_n337, new_n338, new_n339, new_n340,
    new_n341, new_n344, new_n345, new_n346, new_n347, new_n348, new_n349,
    new_n350, new_n351, new_n352, new_n353, new_n354, new_n355, new_n357,
    new_n358, new_n359, new_n360, new_n361, new_n362, new_n363, new_n364,
    new_n366, new_n367, new_n368, new_n370, new_n371, new_n373, new_n374,
    new_n375, new_n377, new_n378, new_n379, new_n381, new_n382, new_n383,
    new_n384, new_n385, new_n387, new_n388, new_n389, new_n391, new_n392;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  and002aa1n12x5               g001(.a(\b[0] ), .b(\a[1] ), .o(new_n97));
  oaoi03aa1n12x5               g002(.a(\a[2] ), .b(\b[1] ), .c(new_n97), .o1(new_n98));
  nor042aa1n06x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  norb02aa1n03x5               g005(.a(new_n100), .b(new_n99), .out0(new_n101));
  nor002aa1d32x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand02aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  norb02aa1n03x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nand03aa1n04x5               g009(.a(new_n98), .b(new_n101), .c(new_n104), .o1(new_n105));
  aoi012aa1n06x5               g010(.a(new_n99), .b(new_n102), .c(new_n100), .o1(new_n106));
  nor022aa1n16x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  nand02aa1d28x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nor042aa1n06x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand02aa1n20x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nano23aa1n06x5               g015(.a(new_n107), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n111));
  nor042aa1d18x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  aoi012aa1n02x5               g017(.a(new_n112), .b(\a[6] ), .c(\b[5] ), .o1(new_n113));
  nor042aa1n06x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  tech160nm_fiaoi012aa1n04x5   g019(.a(new_n114), .b(\a[5] ), .c(\b[4] ), .o1(new_n115));
  nand03aa1n02x5               g020(.a(new_n111), .b(new_n113), .c(new_n115), .o1(new_n116));
  inv040aa1n08x5               g021(.a(new_n112), .o1(new_n117));
  oaoi03aa1n12x5               g022(.a(\a[6] ), .b(\b[5] ), .c(new_n117), .o1(new_n118));
  tech160nm_fiao0012aa1n02p5x5 g023(.a(new_n107), .b(new_n109), .c(new_n108), .o(new_n119));
  aoi012aa1n12x5               g024(.a(new_n119), .b(new_n111), .c(new_n118), .o1(new_n120));
  aoai13aa1n12x5               g025(.a(new_n120), .b(new_n116), .c(new_n105), .d(new_n106), .o1(new_n121));
  nor002aa1d32x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  nand02aa1d28x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n02x5               g028(.a(new_n123), .b(new_n122), .out0(new_n124));
  nor042aa1d18x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand02aa1d28x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanb02aa1n02x5               g031(.a(new_n125), .b(new_n126), .out0(new_n127));
  aoai13aa1n03x5               g032(.a(new_n127), .b(new_n122), .c(new_n121), .d(new_n123), .o1(new_n128));
  nona22aa1n02x4               g033(.a(new_n126), .b(new_n125), .c(new_n122), .out0(new_n129));
  aoai13aa1n03x5               g034(.a(new_n128), .b(new_n129), .c(new_n124), .d(new_n121), .o1(\s[10] ));
  nano23aa1n09x5               g035(.a(new_n122), .b(new_n125), .c(new_n126), .d(new_n123), .out0(new_n131));
  tech160nm_fiaoi012aa1n05x5   g036(.a(new_n125), .b(new_n122), .c(new_n126), .o1(new_n132));
  inv000aa1n02x5               g037(.a(new_n132), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1d28x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoai13aa1n04x5               g041(.a(new_n136), .b(new_n133), .c(new_n121), .d(new_n131), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n136), .b(new_n125), .c(new_n126), .d(new_n122), .o1(new_n138));
  aobi12aa1n02x5               g043(.a(new_n138), .b(new_n121), .c(new_n131), .out0(new_n139));
  norb02aa1n03x4               g044(.a(new_n137), .b(new_n139), .out0(\s[11] ));
  inv000aa1d42x5               g045(.a(new_n134), .o1(new_n141));
  norp02aa1n24x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand02aa1d16x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  nona23aa1n03x5               g049(.a(new_n137), .b(new_n143), .c(new_n142), .d(new_n134), .out0(new_n145));
  aoai13aa1n03x5               g050(.a(new_n145), .b(new_n144), .c(new_n141), .d(new_n137), .o1(\s[12] ));
  nano23aa1n09x5               g051(.a(new_n134), .b(new_n142), .c(new_n143), .d(new_n135), .out0(new_n147));
  nand02aa1d04x5               g052(.a(new_n147), .b(new_n131), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n121), .b(new_n149), .o1(new_n150));
  nano23aa1n03x7               g055(.a(new_n99), .b(new_n102), .c(new_n103), .d(new_n100), .out0(new_n151));
  inv000aa1n02x5               g056(.a(new_n106), .o1(new_n152));
  nona23aa1n03x5               g057(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n153));
  nanp02aa1n02x5               g058(.a(new_n115), .b(new_n113), .o1(new_n154));
  nor042aa1n04x5               g059(.a(new_n153), .b(new_n154), .o1(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n152), .c(new_n98), .d(new_n151), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n142), .b(new_n134), .c(new_n143), .o1(new_n157));
  aobi12aa1n02x5               g062(.a(new_n157), .b(new_n147), .c(new_n133), .out0(new_n158));
  aoai13aa1n04x5               g063(.a(new_n158), .b(new_n148), .c(new_n156), .d(new_n120), .o1(new_n159));
  xnrc02aa1n12x5               g064(.a(\b[12] ), .b(\a[13] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nanb03aa1n02x5               g066(.a(new_n142), .b(new_n143), .c(new_n134), .out0(new_n162));
  oai112aa1n02x5               g067(.a(new_n162), .b(new_n160), .c(\b[11] ), .d(\a[12] ), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n133), .c(new_n147), .o1(new_n164));
  aoi022aa1n02x5               g069(.a(new_n159), .b(new_n161), .c(new_n150), .d(new_n164), .o1(\s[13] ));
  nor042aa1d18x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  inv040aa1d32x5               g071(.a(\a[14] ), .o1(new_n167));
  inv040aa1d32x5               g072(.a(\b[13] ), .o1(new_n168));
  nand02aa1d08x5               g073(.a(new_n168), .b(new_n167), .o1(new_n169));
  nand42aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand22aa1n09x5               g075(.a(new_n169), .b(new_n170), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n166), .c(new_n159), .d(new_n161), .o1(new_n172));
  oai112aa1n02x5               g077(.a(new_n169), .b(new_n170), .c(\b[12] ), .d(\a[13] ), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n173), .c(new_n161), .d(new_n159), .o1(\s[14] ));
  nona23aa1n09x5               g079(.a(new_n143), .b(new_n135), .c(new_n134), .d(new_n142), .out0(new_n175));
  oai012aa1n02x7               g080(.a(new_n157), .b(new_n175), .c(new_n132), .o1(new_n176));
  nor042aa1n06x5               g081(.a(new_n160), .b(new_n171), .o1(new_n177));
  aoai13aa1n03x5               g082(.a(new_n177), .b(new_n176), .c(new_n121), .d(new_n149), .o1(new_n178));
  oaoi03aa1n12x5               g083(.a(new_n167), .b(new_n168), .c(new_n166), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  nor002aa1d32x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  nand42aa1n04x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  norb02aa1n15x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  aoai13aa1n03x5               g088(.a(new_n183), .b(new_n180), .c(new_n159), .d(new_n177), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n169), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n183), .b(new_n185), .c(new_n170), .d(new_n166), .o1(new_n186));
  aobi12aa1n02x7               g091(.a(new_n184), .b(new_n186), .c(new_n178), .out0(\s[15] ));
  inv000aa1d42x5               g092(.a(new_n181), .o1(new_n188));
  nor022aa1n16x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nand42aa1n04x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n183), .o1(new_n192));
  norb03aa1n02x5               g097(.a(new_n190), .b(new_n181), .c(new_n189), .out0(new_n193));
  aoai13aa1n03x5               g098(.a(new_n193), .b(new_n192), .c(new_n178), .d(new_n179), .o1(new_n194));
  aoai13aa1n03x5               g099(.a(new_n194), .b(new_n191), .c(new_n184), .d(new_n188), .o1(\s[16] ));
  nona23aa1n09x5               g100(.a(new_n190), .b(new_n182), .c(new_n181), .d(new_n189), .out0(new_n196));
  nanb02aa1n02x5               g101(.a(new_n196), .b(new_n177), .out0(new_n197));
  nor042aa1n02x5               g102(.a(new_n197), .b(new_n148), .o1(new_n198));
  nand22aa1n03x5               g103(.a(new_n121), .b(new_n198), .o1(new_n199));
  nona23aa1n02x4               g104(.a(new_n177), .b(new_n131), .c(new_n196), .d(new_n175), .out0(new_n200));
  nor043aa1n02x5               g105(.a(new_n196), .b(new_n171), .c(new_n160), .o1(new_n201));
  aoi012aa1n02x5               g106(.a(new_n189), .b(new_n181), .c(new_n190), .o1(new_n202));
  oaih12aa1n02x5               g107(.a(new_n202), .b(new_n196), .c(new_n179), .o1(new_n203));
  aoi012aa1n06x5               g108(.a(new_n203), .b(new_n176), .c(new_n201), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n200), .c(new_n156), .d(new_n120), .o1(new_n205));
  xorc02aa1n12x5               g110(.a(\a[17] ), .b(\b[16] ), .out0(new_n206));
  aoi112aa1n02x5               g111(.a(new_n206), .b(new_n189), .c(new_n190), .d(new_n181), .o1(new_n207));
  oai012aa1n02x5               g112(.a(new_n207), .b(new_n196), .c(new_n179), .o1(new_n208));
  aoi012aa1n02x5               g113(.a(new_n208), .b(new_n176), .c(new_n201), .o1(new_n209));
  aoi022aa1n02x5               g114(.a(new_n205), .b(new_n206), .c(new_n199), .d(new_n209), .o1(\s[17] ));
  aobi12aa1n03x5               g115(.a(new_n206), .b(new_n199), .c(new_n204), .out0(new_n211));
  nor042aa1d18x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  aoi012aa1n02x5               g117(.a(new_n212), .b(new_n205), .c(new_n206), .o1(new_n213));
  nor042aa1n12x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nand02aa1d28x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  norb02aa1n03x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  nona22aa1n02x4               g121(.a(new_n215), .b(new_n214), .c(new_n212), .out0(new_n217));
  oai022aa1n03x5               g122(.a(new_n213), .b(new_n216), .c(new_n211), .d(new_n217), .o1(\s[18] ));
  oabi12aa1n06x5               g123(.a(new_n203), .b(new_n158), .c(new_n197), .out0(new_n219));
  and002aa1n02x5               g124(.a(new_n206), .b(new_n216), .o(new_n220));
  aoai13aa1n03x5               g125(.a(new_n220), .b(new_n219), .c(new_n121), .d(new_n198), .o1(new_n221));
  aoi012aa1d24x5               g126(.a(new_n214), .b(new_n212), .c(new_n215), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  xorc02aa1n06x5               g128(.a(\a[19] ), .b(\b[18] ), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n223), .c(new_n205), .d(new_n220), .o1(new_n225));
  aoi112aa1n02x5               g130(.a(new_n224), .b(new_n214), .c(new_n215), .d(new_n212), .o1(new_n226));
  aobi12aa1n02x7               g131(.a(new_n225), .b(new_n226), .c(new_n221), .out0(\s[19] ));
  xnrc02aa1n02x5               g132(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g133(.a(\b[18] ), .b(\a[19] ), .o1(new_n229));
  inv000aa1n04x5               g134(.a(new_n229), .o1(new_n230));
  nor042aa1n04x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  and002aa1n12x5               g136(.a(\b[19] ), .b(\a[20] ), .o(new_n232));
  nor042aa1n02x5               g137(.a(new_n232), .b(new_n231), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[18] ), .b(\a[19] ), .out0(new_n234));
  norp03aa1n02x5               g139(.a(new_n232), .b(new_n231), .c(new_n229), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n234), .c(new_n221), .d(new_n222), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n233), .c(new_n225), .d(new_n230), .o1(\s[20] ));
  nano32aa1n02x4               g142(.a(new_n234), .b(new_n206), .c(new_n233), .d(new_n216), .out0(new_n238));
  aoai13aa1n03x5               g143(.a(new_n238), .b(new_n219), .c(new_n121), .d(new_n198), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[19] ), .b(\a[20] ), .out0(new_n240));
  oab012aa1n06x5               g145(.a(new_n231), .b(new_n230), .c(new_n232), .out0(new_n241));
  oai013aa1d12x5               g146(.a(new_n241), .b(new_n234), .c(new_n240), .d(new_n222), .o1(new_n242));
  nor002aa1d32x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  nand02aa1n16x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  nanb02aa1n12x5               g149(.a(new_n243), .b(new_n244), .out0(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n242), .c(new_n205), .d(new_n238), .o1(new_n247));
  nanb03aa1n03x5               g152(.a(new_n222), .b(new_n224), .c(new_n233), .out0(new_n248));
  nona22aa1n02x4               g153(.a(new_n229), .b(new_n232), .c(new_n231), .out0(new_n249));
  nano32aa1n02x4               g154(.a(new_n231), .b(new_n245), .c(new_n248), .d(new_n249), .out0(new_n250));
  aobi12aa1n02x7               g155(.a(new_n247), .b(new_n250), .c(new_n239), .out0(\s[21] ));
  inv000aa1d42x5               g156(.a(new_n243), .o1(new_n252));
  nor002aa1d32x5               g157(.a(\b[21] ), .b(\a[22] ), .o1(new_n253));
  nand02aa1d28x5               g158(.a(\b[21] ), .b(\a[22] ), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n254), .b(new_n253), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n242), .o1(new_n256));
  norb03aa1n02x5               g161(.a(new_n254), .b(new_n243), .c(new_n253), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n245), .c(new_n239), .d(new_n256), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n255), .c(new_n247), .d(new_n252), .o1(\s[22] ));
  norp02aa1n02x5               g164(.a(new_n240), .b(new_n234), .o1(new_n260));
  nona23aa1d18x5               g165(.a(new_n254), .b(new_n244), .c(new_n243), .d(new_n253), .out0(new_n261));
  nano32aa1n02x4               g166(.a(new_n261), .b(new_n260), .c(new_n216), .d(new_n206), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n219), .c(new_n121), .d(new_n198), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n261), .o1(new_n264));
  aoi012aa1n06x5               g169(.a(new_n253), .b(new_n243), .c(new_n254), .o1(new_n265));
  inv020aa1n03x5               g170(.a(new_n265), .o1(new_n266));
  aoi012aa1n09x5               g171(.a(new_n266), .b(new_n242), .c(new_n264), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n267), .o1(new_n268));
  nor002aa1d32x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  nand02aa1d24x5               g174(.a(\b[22] ), .b(\a[23] ), .o1(new_n270));
  norb02aa1n09x5               g175(.a(new_n270), .b(new_n269), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n268), .c(new_n205), .d(new_n262), .o1(new_n272));
  aoi112aa1n02x5               g177(.a(new_n271), .b(new_n253), .c(new_n254), .d(new_n243), .o1(new_n273));
  aobi12aa1n02x5               g178(.a(new_n273), .b(new_n242), .c(new_n264), .out0(new_n274));
  aobi12aa1n02x7               g179(.a(new_n272), .b(new_n274), .c(new_n263), .out0(\s[23] ));
  inv000aa1n02x5               g180(.a(new_n269), .o1(new_n276));
  nor002aa1n12x5               g181(.a(\b[23] ), .b(\a[24] ), .o1(new_n277));
  nand02aa1d16x5               g182(.a(\b[23] ), .b(\a[24] ), .o1(new_n278));
  norb02aa1n02x5               g183(.a(new_n278), .b(new_n277), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n271), .o1(new_n280));
  norb03aa1n02x5               g185(.a(new_n278), .b(new_n269), .c(new_n277), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n263), .d(new_n267), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n282), .b(new_n279), .c(new_n272), .d(new_n276), .o1(\s[24] ));
  nano23aa1n03x7               g188(.a(new_n269), .b(new_n277), .c(new_n278), .d(new_n270), .out0(new_n284));
  nanp03aa1n02x5               g189(.a(new_n284), .b(new_n246), .c(new_n255), .o1(new_n285));
  nano32aa1n02x4               g190(.a(new_n285), .b(new_n260), .c(new_n216), .d(new_n206), .out0(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n219), .c(new_n121), .d(new_n198), .o1(new_n287));
  nano22aa1n06x5               g192(.a(new_n261), .b(new_n271), .c(new_n279), .out0(new_n288));
  nanp02aa1n03x5               g193(.a(new_n242), .b(new_n288), .o1(new_n289));
  oaoi03aa1n02x5               g194(.a(\a[24] ), .b(\b[23] ), .c(new_n276), .o1(new_n290));
  aoi012aa1n06x5               g195(.a(new_n290), .b(new_n284), .c(new_n266), .o1(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n285), .c(new_n248), .d(new_n241), .o1(new_n292));
  inv000aa1n02x5               g197(.a(new_n292), .o1(new_n293));
  xorc02aa1n12x5               g198(.a(\a[25] ), .b(\b[24] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  aoi012aa1n03x5               g200(.a(new_n295), .b(new_n287), .c(new_n293), .o1(new_n296));
  aoi112aa1n02x5               g201(.a(new_n294), .b(new_n277), .c(new_n278), .d(new_n269), .o1(new_n297));
  aobi12aa1n02x5               g202(.a(new_n297), .b(new_n284), .c(new_n266), .out0(new_n298));
  aoi013aa1n02x4               g203(.a(new_n296), .b(new_n289), .c(new_n287), .d(new_n298), .o1(\s[25] ));
  nor042aa1n03x5               g204(.a(\b[24] ), .b(\a[25] ), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  aoai13aa1n02x5               g206(.a(new_n294), .b(new_n292), .c(new_n205), .d(new_n286), .o1(new_n302));
  nor002aa1n12x5               g207(.a(\b[25] ), .b(\a[26] ), .o1(new_n303));
  and002aa1n12x5               g208(.a(\b[25] ), .b(\a[26] ), .o(new_n304));
  nor042aa1n02x5               g209(.a(new_n304), .b(new_n303), .o1(new_n305));
  norp03aa1n02x5               g210(.a(new_n304), .b(new_n303), .c(new_n300), .o1(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n295), .c(new_n287), .d(new_n293), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n305), .c(new_n302), .d(new_n301), .o1(\s[26] ));
  and002aa1n03x5               g213(.a(new_n294), .b(new_n305), .o(new_n309));
  inv000aa1n02x5               g214(.a(new_n309), .o1(new_n310));
  nano32aa1n03x7               g215(.a(new_n310), .b(new_n220), .c(new_n288), .d(new_n260), .out0(new_n311));
  aoai13aa1n06x5               g216(.a(new_n311), .b(new_n219), .c(new_n121), .d(new_n198), .o1(new_n312));
  oab012aa1n02x4               g217(.a(new_n303), .b(new_n301), .c(new_n304), .out0(new_n313));
  aoai13aa1n02x7               g218(.a(new_n313), .b(new_n310), .c(new_n289), .d(new_n291), .o1(new_n314));
  xorc02aa1n12x5               g219(.a(\a[27] ), .b(\b[26] ), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n314), .c(new_n205), .d(new_n311), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n303), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n315), .o1(new_n318));
  oai112aa1n02x5               g223(.a(new_n318), .b(new_n317), .c(new_n304), .d(new_n301), .o1(new_n319));
  aoi012aa1n02x5               g224(.a(new_n319), .b(new_n292), .c(new_n309), .o1(new_n320));
  aobi12aa1n02x7               g225(.a(new_n316), .b(new_n320), .c(new_n312), .out0(\s[27] ));
  nor042aa1n06x5               g226(.a(\b[26] ), .b(\a[27] ), .o1(new_n322));
  inv000aa1n04x5               g227(.a(new_n322), .o1(new_n323));
  nor042aa1n04x5               g228(.a(\b[27] ), .b(\a[28] ), .o1(new_n324));
  and002aa1n12x5               g229(.a(\b[27] ), .b(\a[28] ), .o(new_n325));
  nor002aa1n02x5               g230(.a(new_n325), .b(new_n324), .o1(new_n326));
  aobi12aa1n06x5               g231(.a(new_n313), .b(new_n292), .c(new_n309), .out0(new_n327));
  norp03aa1n02x5               g232(.a(new_n325), .b(new_n324), .c(new_n322), .o1(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n318), .c(new_n312), .d(new_n327), .o1(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n326), .c(new_n316), .d(new_n323), .o1(\s[28] ));
  inv000aa1d42x5               g235(.a(\a[28] ), .o1(new_n331));
  inv000aa1d42x5               g236(.a(\b[26] ), .o1(new_n332));
  xroi22aa1d04x5               g237(.a(\a[27] ), .b(new_n332), .c(new_n331), .d(\b[27] ), .out0(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n314), .c(new_n205), .d(new_n311), .o1(new_n334));
  inv000aa1d42x5               g239(.a(new_n333), .o1(new_n335));
  norp03aa1n02x5               g240(.a(new_n323), .b(new_n325), .c(new_n324), .o1(new_n336));
  oai022aa1n02x5               g241(.a(\a[28] ), .b(\b[27] ), .c(\b[28] ), .d(\a[29] ), .o1(new_n337));
  aoi112aa1n02x5               g242(.a(new_n336), .b(new_n337), .c(\a[29] ), .d(\b[28] ), .o1(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n335), .c(new_n312), .d(new_n327), .o1(new_n339));
  oab012aa1n06x5               g244(.a(new_n324), .b(new_n323), .c(new_n325), .out0(new_n340));
  xorc02aa1n02x5               g245(.a(\a[29] ), .b(\b[28] ), .out0(new_n341));
  aoai13aa1n03x5               g246(.a(new_n339), .b(new_n341), .c(new_n334), .d(new_n340), .o1(\s[29] ));
  xnrb03aa1n02x5               g247(.a(new_n97), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g248(.a(new_n318), .b(new_n341), .c(new_n326), .out0(new_n344));
  aoai13aa1n02x5               g249(.a(new_n344), .b(new_n314), .c(new_n205), .d(new_n311), .o1(new_n345));
  tech160nm_fioaoi03aa1n03p5x5 g250(.a(\a[29] ), .b(\b[28] ), .c(new_n340), .o1(new_n346));
  inv000aa1d42x5               g251(.a(new_n346), .o1(new_n347));
  norp02aa1n02x5               g252(.a(\b[29] ), .b(\a[30] ), .o1(new_n348));
  nanp02aa1n02x5               g253(.a(\b[29] ), .b(\a[30] ), .o1(new_n349));
  norb02aa1n02x5               g254(.a(new_n349), .b(new_n348), .out0(new_n350));
  inv000aa1d42x5               g255(.a(new_n344), .o1(new_n351));
  oai012aa1n02x5               g256(.a(new_n341), .b(new_n336), .c(new_n324), .o1(new_n352));
  oai022aa1n02x5               g257(.a(\a[29] ), .b(\b[28] ), .c(\b[29] ), .d(\a[30] ), .o1(new_n353));
  nano22aa1n02x4               g258(.a(new_n353), .b(new_n352), .c(new_n349), .out0(new_n354));
  aoai13aa1n03x5               g259(.a(new_n354), .b(new_n351), .c(new_n312), .d(new_n327), .o1(new_n355));
  aoai13aa1n03x5               g260(.a(new_n355), .b(new_n350), .c(new_n345), .d(new_n347), .o1(\s[30] ));
  nano32aa1n03x7               g261(.a(new_n318), .b(new_n350), .c(new_n326), .d(new_n341), .out0(new_n357));
  aoai13aa1n02x5               g262(.a(new_n357), .b(new_n314), .c(new_n205), .d(new_n311), .o1(new_n358));
  xnrc02aa1n02x5               g263(.a(\b[30] ), .b(\a[31] ), .out0(new_n359));
  inv000aa1d42x5               g264(.a(new_n359), .o1(new_n360));
  inv000aa1d42x5               g265(.a(new_n357), .o1(new_n361));
  aoi112aa1n02x5               g266(.a(new_n348), .b(new_n359), .c(new_n346), .d(new_n350), .o1(new_n362));
  aoai13aa1n03x5               g267(.a(new_n362), .b(new_n361), .c(new_n312), .d(new_n327), .o1(new_n363));
  aoi012aa1n02x5               g268(.a(new_n348), .b(new_n346), .c(new_n349), .o1(new_n364));
  aoai13aa1n03x5               g269(.a(new_n363), .b(new_n360), .c(new_n358), .d(new_n364), .o1(\s[31] ));
  orn002aa1n02x5               g270(.a(\a[2] ), .b(\b[1] ), .o(new_n366));
  nanp02aa1n02x5               g271(.a(\b[1] ), .b(\a[2] ), .o1(new_n367));
  nanb03aa1n02x5               g272(.a(new_n97), .b(new_n366), .c(new_n367), .out0(new_n368));
  xnbna2aa1n03x5               g273(.a(new_n104), .b(new_n368), .c(new_n366), .out0(\s[3] ));
  inv000aa1d42x5               g274(.a(new_n102), .o1(new_n370));
  nanp02aa1n02x5               g275(.a(new_n98), .b(new_n104), .o1(new_n371));
  xnbna2aa1n03x5               g276(.a(new_n101), .b(new_n371), .c(new_n370), .out0(\s[4] ));
  nanp02aa1n03x5               g277(.a(new_n105), .b(new_n106), .o1(new_n373));
  xorc02aa1n02x5               g278(.a(\a[5] ), .b(\b[4] ), .out0(new_n374));
  aoi112aa1n02x5               g279(.a(new_n374), .b(new_n99), .c(new_n100), .d(new_n102), .o1(new_n375));
  aoi022aa1n02x5               g280(.a(new_n373), .b(new_n374), .c(new_n105), .d(new_n375), .o1(\s[5] ));
  aoai13aa1n02x5               g281(.a(new_n374), .b(new_n152), .c(new_n151), .d(new_n98), .o1(new_n377));
  nanp02aa1n02x5               g282(.a(\b[5] ), .b(\a[6] ), .o1(new_n378));
  nanb02aa1n02x5               g283(.a(new_n114), .b(new_n378), .out0(new_n379));
  xobna2aa1n03x5               g284(.a(new_n379), .b(new_n377), .c(new_n117), .out0(\s[6] ));
  nanb03aa1n06x5               g285(.a(new_n379), .b(new_n373), .c(new_n374), .out0(new_n381));
  norb02aa1n02x5               g286(.a(new_n110), .b(new_n109), .out0(new_n382));
  inv000aa1d42x5               g287(.a(new_n118), .o1(new_n383));
  aobi12aa1n06x5               g288(.a(new_n382), .b(new_n381), .c(new_n383), .out0(new_n384));
  aoi112aa1n02x5               g289(.a(new_n382), .b(new_n114), .c(new_n378), .d(new_n112), .o1(new_n385));
  aoi012aa1n02x5               g290(.a(new_n384), .b(new_n381), .c(new_n385), .o1(\s[7] ));
  norb02aa1n02x5               g291(.a(new_n108), .b(new_n107), .out0(new_n387));
  oabi12aa1n03x5               g292(.a(new_n387), .b(new_n384), .c(new_n109), .out0(new_n388));
  norb03aa1n02x5               g293(.a(new_n108), .b(new_n107), .c(new_n109), .out0(new_n389));
  oaib12aa1n03x5               g294(.a(new_n388), .b(new_n384), .c(new_n389), .out0(\s[8] ));
  aoi112aa1n02x5               g295(.a(new_n124), .b(new_n107), .c(new_n108), .d(new_n109), .o1(new_n391));
  aobi12aa1n02x5               g296(.a(new_n391), .b(new_n118), .c(new_n111), .out0(new_n392));
  aoi022aa1n02x5               g297(.a(new_n121), .b(new_n124), .c(new_n156), .d(new_n392), .o1(\s[9] ));
endmodule


