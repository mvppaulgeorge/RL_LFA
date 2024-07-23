// Benchmark "adder" written by ABC on Wed Jul 17 15:54:49 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n303, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n356, new_n357, new_n358,
    new_n359, new_n360, new_n362, new_n363, new_n366, new_n367, new_n369,
    new_n371, new_n372, new_n373, new_n375, new_n376;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand02aa1d28x5               g001(.a(\b[0] ), .b(\a[1] ), .o1(new_n97));
  inv000aa1n04x5               g002(.a(new_n97), .o1(new_n98));
  oaoi03aa1n12x5               g003(.a(\a[2] ), .b(\b[1] ), .c(new_n98), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nanp02aa1n09x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  norb02aa1n03x5               g006(.a(new_n101), .b(new_n100), .out0(new_n102));
  nor002aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand02aa1d06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n09x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nand23aa1n04x5               g010(.a(new_n99), .b(new_n102), .c(new_n105), .o1(new_n106));
  tech160nm_fiaoi012aa1n04x5   g011(.a(new_n100), .b(new_n103), .c(new_n101), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  tech160nm_finand02aa1n03p5x5 g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nor022aa1n08x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nano23aa1n03x7               g016(.a(new_n111), .b(new_n108), .c(new_n109), .d(new_n110), .out0(new_n112));
  xorc02aa1n12x5               g017(.a(\a[8] ), .b(\b[7] ), .out0(new_n113));
  tech160nm_finand02aa1n05x5   g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor042aa1n04x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  nanp03aa1n03x5               g021(.a(new_n112), .b(new_n113), .c(new_n116), .o1(new_n117));
  nano22aa1n03x7               g022(.a(new_n115), .b(new_n109), .c(new_n114), .out0(new_n118));
  norp02aa1n04x5               g023(.a(new_n111), .b(new_n108), .o1(new_n119));
  inv040aa1n03x5               g024(.a(new_n119), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[8] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[7] ), .o1(new_n122));
  oaoi03aa1n06x5               g027(.a(new_n121), .b(new_n122), .c(new_n115), .o1(new_n123));
  inv000aa1n02x5               g028(.a(new_n123), .o1(new_n124));
  aoi013aa1n06x4               g029(.a(new_n124), .b(new_n118), .c(new_n113), .d(new_n120), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n125), .b(new_n117), .c(new_n106), .d(new_n107), .o1(new_n126));
  nor042aa1n03x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  xorc02aa1n12x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  nor042aa1d18x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n03x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n06x4               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  aoi112aa1n02x5               g036(.a(new_n127), .b(new_n131), .c(new_n126), .d(new_n128), .o1(new_n132));
  aoai13aa1n06x5               g037(.a(new_n131), .b(new_n127), .c(new_n126), .d(new_n128), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(\s[10] ));
  nor002aa1n16x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n12x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  inv000aa1d42x5               g042(.a(new_n129), .o1(new_n138));
  aob012aa1n03x5               g043(.a(new_n137), .b(new_n133), .c(new_n138), .out0(new_n139));
  nand02aa1n03x5               g044(.a(new_n106), .b(new_n107), .o1(new_n140));
  norb02aa1n03x5               g045(.a(new_n109), .b(new_n108), .out0(new_n141));
  norb02aa1n06x4               g046(.a(new_n110), .b(new_n111), .out0(new_n142));
  xnrc02aa1n02x5               g047(.a(\b[7] ), .b(\a[8] ), .out0(new_n143));
  nano32aa1n03x7               g048(.a(new_n143), .b(new_n116), .c(new_n141), .d(new_n142), .out0(new_n144));
  nanp03aa1n02x5               g049(.a(new_n118), .b(new_n113), .c(new_n120), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(new_n145), .b(new_n123), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n128), .b(new_n146), .c(new_n140), .d(new_n144), .o1(new_n147));
  nor042aa1n02x5               g052(.a(new_n129), .b(new_n127), .o1(new_n148));
  aoi022aa1n02x5               g053(.a(new_n147), .b(new_n148), .c(\b[9] ), .d(\a[10] ), .o1(new_n149));
  oa0012aa1n03x5               g054(.a(new_n139), .b(new_n149), .c(new_n137), .o(\s[11] ));
  nor042aa1n04x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nand42aa1n08x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n152), .b(new_n151), .out0(new_n153));
  aoib12aa1n02x5               g058(.a(new_n135), .b(new_n152), .c(new_n151), .out0(new_n154));
  inv000aa1d42x5               g059(.a(new_n135), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n137), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n155), .b(new_n156), .c(new_n133), .d(new_n138), .o1(new_n157));
  aoi022aa1n03x5               g062(.a(new_n157), .b(new_n153), .c(new_n139), .d(new_n154), .o1(\s[12] ));
  nano32aa1n06x5               g063(.a(new_n156), .b(new_n128), .c(new_n153), .d(new_n131), .out0(new_n159));
  aoai13aa1n03x5               g064(.a(new_n159), .b(new_n146), .c(new_n140), .d(new_n144), .o1(new_n160));
  nano22aa1n03x7               g065(.a(new_n151), .b(new_n136), .c(new_n152), .out0(new_n161));
  nona23aa1n09x5               g066(.a(new_n161), .b(new_n130), .c(new_n148), .d(new_n135), .out0(new_n162));
  aoi012aa1n09x5               g067(.a(new_n151), .b(new_n135), .c(new_n152), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(new_n162), .b(new_n163), .o1(new_n164));
  nanb02aa1n03x5               g069(.a(new_n164), .b(new_n160), .out0(new_n165));
  xorc02aa1n12x5               g070(.a(\a[13] ), .b(\b[12] ), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  and003aa1n02x5               g072(.a(new_n162), .b(new_n167), .c(new_n163), .o(new_n168));
  aoi022aa1n02x5               g073(.a(new_n165), .b(new_n166), .c(new_n160), .d(new_n168), .o1(\s[13] ));
  orn002aa1n02x5               g074(.a(\a[13] ), .b(\b[12] ), .o(new_n170));
  aoai13aa1n02x5               g075(.a(new_n166), .b(new_n164), .c(new_n126), .d(new_n159), .o1(new_n171));
  xorc02aa1n12x5               g076(.a(\a[14] ), .b(\b[13] ), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n171), .c(new_n170), .out0(\s[14] ));
  and002aa1n02x5               g078(.a(new_n172), .b(new_n166), .o(new_n174));
  aoai13aa1n03x5               g079(.a(new_n174), .b(new_n164), .c(new_n126), .d(new_n159), .o1(new_n175));
  oaoi03aa1n02x5               g080(.a(\a[14] ), .b(\b[13] ), .c(new_n170), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  nor042aa1n09x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand42aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n02x7               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n175), .c(new_n177), .out0(\s[15] ));
  aoai13aa1n06x5               g086(.a(new_n180), .b(new_n176), .c(new_n165), .d(new_n174), .o1(new_n182));
  nor042aa1n04x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nand02aa1n03x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  norp02aa1n02x5               g090(.a(new_n185), .b(new_n178), .o1(new_n186));
  inv000aa1d42x5               g091(.a(new_n178), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n180), .o1(new_n188));
  aoai13aa1n06x5               g093(.a(new_n187), .b(new_n188), .c(new_n175), .d(new_n177), .o1(new_n189));
  aoi022aa1n03x5               g094(.a(new_n189), .b(new_n185), .c(new_n182), .d(new_n186), .o1(\s[16] ));
  nano23aa1n06x5               g095(.a(new_n135), .b(new_n151), .c(new_n152), .d(new_n136), .out0(new_n191));
  nano23aa1n06x5               g096(.a(new_n178), .b(new_n183), .c(new_n184), .d(new_n179), .out0(new_n192));
  nand23aa1n09x5               g097(.a(new_n192), .b(new_n166), .c(new_n172), .o1(new_n193));
  nano32aa1d12x5               g098(.a(new_n193), .b(new_n191), .c(new_n131), .d(new_n128), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n146), .c(new_n140), .d(new_n144), .o1(new_n195));
  nanb03aa1n02x5               g100(.a(new_n183), .b(new_n184), .c(new_n179), .out0(new_n196));
  inv000aa1d42x5               g101(.a(\a[14] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(\b[13] ), .o1(new_n198));
  oai022aa1n02x5               g103(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n199));
  oai122aa1n02x7               g104(.a(new_n199), .b(\a[15] ), .c(\b[14] ), .d(new_n197), .e(new_n198), .o1(new_n200));
  nor042aa1n02x5               g105(.a(new_n200), .b(new_n196), .o1(new_n201));
  aoi012aa1n02x5               g106(.a(new_n183), .b(new_n178), .c(new_n184), .o1(new_n202));
  norb02aa1n09x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  aoai13aa1n12x5               g108(.a(new_n203), .b(new_n193), .c(new_n162), .d(new_n163), .o1(new_n204));
  inv040aa1n06x5               g109(.a(new_n204), .o1(new_n205));
  nand02aa1d08x5               g110(.a(new_n195), .b(new_n205), .o1(new_n206));
  xorc02aa1n12x5               g111(.a(\a[17] ), .b(\b[16] ), .out0(new_n207));
  nona22aa1n02x4               g112(.a(new_n202), .b(new_n201), .c(new_n207), .out0(new_n208));
  aoib12aa1n02x5               g113(.a(new_n208), .b(new_n164), .c(new_n193), .out0(new_n209));
  aoi022aa1n02x5               g114(.a(new_n206), .b(new_n207), .c(new_n195), .d(new_n209), .o1(\s[17] ));
  nor002aa1d32x5               g115(.a(\b[16] ), .b(\a[17] ), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n207), .b(new_n204), .c(new_n126), .d(new_n194), .o1(new_n213));
  nor042aa1d18x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nand02aa1d28x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  norb02aa1n03x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n213), .c(new_n212), .out0(\s[18] ));
  and002aa1n02x5               g122(.a(new_n207), .b(new_n216), .o(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n204), .c(new_n126), .d(new_n194), .o1(new_n219));
  oaoi03aa1n02x5               g124(.a(\a[18] ), .b(\b[17] ), .c(new_n212), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  nor042aa1d18x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  nand22aa1n12x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  norb02aa1n12x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  xnbna2aa1n03x5               g129(.a(new_n224), .b(new_n219), .c(new_n221), .out0(\s[19] ));
  xnrc02aa1n02x5               g130(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n06x5               g131(.a(new_n224), .b(new_n220), .c(new_n206), .d(new_n218), .o1(new_n227));
  nor002aa1n16x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  nand02aa1d28x5               g133(.a(\b[19] ), .b(\a[20] ), .o1(new_n229));
  norb02aa1n06x4               g134(.a(new_n229), .b(new_n228), .out0(new_n230));
  inv000aa1d42x5               g135(.a(\a[19] ), .o1(new_n231));
  inv000aa1d42x5               g136(.a(\b[18] ), .o1(new_n232));
  aboi22aa1n03x5               g137(.a(new_n228), .b(new_n229), .c(new_n231), .d(new_n232), .out0(new_n233));
  inv040aa1n03x5               g138(.a(new_n222), .o1(new_n234));
  inv000aa1n02x5               g139(.a(new_n224), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n235), .c(new_n219), .d(new_n221), .o1(new_n236));
  aoi022aa1n03x5               g141(.a(new_n236), .b(new_n230), .c(new_n227), .d(new_n233), .o1(\s[20] ));
  nano32aa1n03x7               g142(.a(new_n235), .b(new_n207), .c(new_n230), .d(new_n216), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n204), .c(new_n126), .d(new_n194), .o1(new_n239));
  nanb03aa1n12x5               g144(.a(new_n228), .b(new_n229), .c(new_n223), .out0(new_n240));
  oai112aa1n06x5               g145(.a(new_n234), .b(new_n215), .c(new_n214), .d(new_n211), .o1(new_n241));
  aoi012aa1d24x5               g146(.a(new_n228), .b(new_n222), .c(new_n229), .o1(new_n242));
  oai012aa1d24x5               g147(.a(new_n242), .b(new_n241), .c(new_n240), .o1(new_n243));
  nor002aa1d32x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  nand42aa1d28x5               g149(.a(\b[20] ), .b(\a[21] ), .o1(new_n245));
  norb02aa1d27x5               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n243), .c(new_n206), .d(new_n238), .o1(new_n247));
  nano22aa1n12x5               g152(.a(new_n228), .b(new_n223), .c(new_n229), .out0(new_n248));
  oai012aa1n02x7               g153(.a(new_n215), .b(\b[18] ), .c(\a[19] ), .o1(new_n249));
  oab012aa1n02x5               g154(.a(new_n249), .b(new_n211), .c(new_n214), .out0(new_n250));
  inv020aa1n02x5               g155(.a(new_n242), .o1(new_n251));
  aoi112aa1n02x5               g156(.a(new_n251), .b(new_n246), .c(new_n250), .d(new_n248), .o1(new_n252));
  aobi12aa1n02x7               g157(.a(new_n247), .b(new_n252), .c(new_n239), .out0(\s[21] ));
  nor042aa1n04x5               g158(.a(\b[21] ), .b(\a[22] ), .o1(new_n254));
  nand42aa1n16x5               g159(.a(\b[21] ), .b(\a[22] ), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n254), .out0(new_n256));
  inv000aa1d42x5               g161(.a(\a[21] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(\b[20] ), .o1(new_n258));
  aboi22aa1n03x5               g163(.a(new_n254), .b(new_n255), .c(new_n257), .d(new_n258), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n243), .o1(new_n260));
  inv040aa1n08x5               g165(.a(new_n244), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n246), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n261), .b(new_n262), .c(new_n239), .d(new_n260), .o1(new_n263));
  aoi022aa1n03x5               g168(.a(new_n263), .b(new_n256), .c(new_n247), .d(new_n259), .o1(\s[22] ));
  nano23aa1d15x5               g169(.a(new_n244), .b(new_n254), .c(new_n255), .d(new_n245), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  nano32aa1n02x5               g171(.a(new_n266), .b(new_n218), .c(new_n224), .d(new_n230), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n267), .b(new_n204), .c(new_n126), .d(new_n194), .o1(new_n268));
  oaoi03aa1n06x5               g173(.a(\a[22] ), .b(\b[21] ), .c(new_n261), .o1(new_n269));
  tech160nm_fiaoi012aa1n03p5x5 g174(.a(new_n269), .b(new_n243), .c(new_n265), .o1(new_n270));
  inv040aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  xorc02aa1n12x5               g176(.a(\a[23] ), .b(\b[22] ), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n271), .c(new_n206), .d(new_n267), .o1(new_n273));
  aoi112aa1n02x5               g178(.a(new_n272), .b(new_n269), .c(new_n243), .d(new_n265), .o1(new_n274));
  aobi12aa1n02x7               g179(.a(new_n273), .b(new_n274), .c(new_n268), .out0(\s[23] ));
  tech160nm_fixorc02aa1n04x5   g180(.a(\a[24] ), .b(\b[23] ), .out0(new_n276));
  nor042aa1n06x5               g181(.a(\b[22] ), .b(\a[23] ), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n276), .b(new_n277), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n277), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n272), .o1(new_n280));
  aoai13aa1n02x7               g185(.a(new_n279), .b(new_n280), .c(new_n268), .d(new_n270), .o1(new_n281));
  aoi022aa1n03x5               g186(.a(new_n281), .b(new_n276), .c(new_n273), .d(new_n278), .o1(\s[24] ));
  nano32aa1n03x7               g187(.a(new_n280), .b(new_n276), .c(new_n246), .d(new_n256), .out0(new_n283));
  and002aa1n02x5               g188(.a(new_n283), .b(new_n238), .o(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n204), .c(new_n126), .d(new_n194), .o1(new_n285));
  aoai13aa1n06x5               g190(.a(new_n265), .b(new_n251), .c(new_n250), .d(new_n248), .o1(new_n286));
  inv000aa1n02x5               g191(.a(new_n269), .o1(new_n287));
  and002aa1n12x5               g192(.a(new_n276), .b(new_n272), .o(new_n288));
  inv000aa1n09x5               g193(.a(new_n288), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[24] ), .b(\b[23] ), .c(new_n279), .carry(new_n290));
  aoai13aa1n12x5               g195(.a(new_n290), .b(new_n289), .c(new_n286), .d(new_n287), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[25] ), .b(\b[24] ), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n291), .c(new_n206), .d(new_n284), .o1(new_n293));
  aoai13aa1n06x5               g198(.a(new_n288), .b(new_n269), .c(new_n243), .d(new_n265), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n292), .o1(new_n295));
  and003aa1n02x5               g200(.a(new_n294), .b(new_n295), .c(new_n290), .o(new_n296));
  aobi12aa1n03x7               g201(.a(new_n293), .b(new_n296), .c(new_n285), .out0(\s[25] ));
  tech160nm_fixorc02aa1n02p5x5 g202(.a(\a[26] ), .b(\b[25] ), .out0(new_n298));
  nor042aa1n03x5               g203(.a(\b[24] ), .b(\a[25] ), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n298), .b(new_n299), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n291), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n299), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n295), .c(new_n285), .d(new_n301), .o1(new_n303));
  aoi022aa1n03x5               g208(.a(new_n303), .b(new_n298), .c(new_n293), .d(new_n300), .o1(\s[26] ));
  and002aa1n12x5               g209(.a(new_n298), .b(new_n292), .o(new_n305));
  and003aa1n18x5               g210(.a(new_n283), .b(new_n238), .c(new_n305), .o(new_n306));
  aoai13aa1n06x5               g211(.a(new_n306), .b(new_n204), .c(new_n126), .d(new_n194), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n305), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[26] ), .b(\b[25] ), .c(new_n302), .carry(new_n309));
  aoai13aa1n06x5               g214(.a(new_n309), .b(new_n308), .c(new_n294), .d(new_n290), .o1(new_n310));
  xorc02aa1n12x5               g215(.a(\a[27] ), .b(\b[26] ), .out0(new_n311));
  aoai13aa1n06x5               g216(.a(new_n311), .b(new_n310), .c(new_n206), .d(new_n306), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n309), .o1(new_n313));
  aoi112aa1n02x5               g218(.a(new_n311), .b(new_n313), .c(new_n291), .d(new_n305), .o1(new_n314));
  aobi12aa1n03x7               g219(.a(new_n312), .b(new_n314), .c(new_n307), .out0(\s[27] ));
  xorc02aa1n02x5               g220(.a(\a[28] ), .b(\b[27] ), .out0(new_n316));
  norp02aa1n02x5               g221(.a(\b[26] ), .b(\a[27] ), .o1(new_n317));
  norp02aa1n02x5               g222(.a(new_n316), .b(new_n317), .o1(new_n318));
  tech160nm_fiaoi012aa1n05x5   g223(.a(new_n313), .b(new_n291), .c(new_n305), .o1(new_n319));
  inv000aa1n03x5               g224(.a(new_n317), .o1(new_n320));
  inv000aa1n02x5               g225(.a(new_n311), .o1(new_n321));
  aoai13aa1n03x5               g226(.a(new_n320), .b(new_n321), .c(new_n319), .d(new_n307), .o1(new_n322));
  aoi022aa1n03x5               g227(.a(new_n322), .b(new_n316), .c(new_n312), .d(new_n318), .o1(\s[28] ));
  and002aa1n02x5               g228(.a(new_n316), .b(new_n311), .o(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n310), .c(new_n206), .d(new_n306), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[29] ), .b(\b[28] ), .out0(new_n326));
  oao003aa1n02x5               g231(.a(\a[28] ), .b(\b[27] ), .c(new_n320), .carry(new_n327));
  norb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(new_n328));
  inv000aa1d42x5               g233(.a(new_n324), .o1(new_n329));
  aoai13aa1n02x7               g234(.a(new_n327), .b(new_n329), .c(new_n319), .d(new_n307), .o1(new_n330));
  aoi022aa1n03x5               g235(.a(new_n330), .b(new_n326), .c(new_n325), .d(new_n328), .o1(\s[29] ));
  xorb03aa1n02x5               g236(.a(new_n97), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g237(.a(new_n321), .b(new_n316), .c(new_n326), .out0(new_n333));
  aoai13aa1n03x5               g238(.a(new_n333), .b(new_n310), .c(new_n206), .d(new_n306), .o1(new_n334));
  inv000aa1n02x5               g239(.a(new_n333), .o1(new_n335));
  inv000aa1d42x5               g240(.a(\b[28] ), .o1(new_n336));
  inv000aa1d42x5               g241(.a(\a[29] ), .o1(new_n337));
  oaib12aa1n02x5               g242(.a(new_n327), .b(\b[28] ), .c(new_n337), .out0(new_n338));
  oaib12aa1n02x5               g243(.a(new_n338), .b(new_n336), .c(\a[29] ), .out0(new_n339));
  aoai13aa1n03x5               g244(.a(new_n339), .b(new_n335), .c(new_n319), .d(new_n307), .o1(new_n340));
  xorc02aa1n02x5               g245(.a(\a[30] ), .b(\b[29] ), .out0(new_n341));
  oaoi13aa1n02x5               g246(.a(new_n341), .b(new_n338), .c(new_n337), .d(new_n336), .o1(new_n342));
  aoi022aa1n03x5               g247(.a(new_n340), .b(new_n341), .c(new_n334), .d(new_n342), .o1(\s[30] ));
  nano32aa1n02x4               g248(.a(new_n321), .b(new_n341), .c(new_n316), .d(new_n326), .out0(new_n344));
  aoai13aa1n03x5               g249(.a(new_n344), .b(new_n310), .c(new_n206), .d(new_n306), .o1(new_n345));
  aoi022aa1n02x5               g250(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n346));
  norb02aa1n02x5               g251(.a(\b[30] ), .b(\a[31] ), .out0(new_n347));
  obai22aa1n02x7               g252(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n348));
  aoi112aa1n02x5               g253(.a(new_n348), .b(new_n347), .c(new_n338), .d(new_n346), .o1(new_n349));
  inv000aa1n02x5               g254(.a(new_n344), .o1(new_n350));
  norp02aa1n02x5               g255(.a(\b[29] ), .b(\a[30] ), .o1(new_n351));
  aoi012aa1n02x5               g256(.a(new_n351), .b(new_n338), .c(new_n346), .o1(new_n352));
  aoai13aa1n02x7               g257(.a(new_n352), .b(new_n350), .c(new_n319), .d(new_n307), .o1(new_n353));
  xorc02aa1n02x5               g258(.a(\a[31] ), .b(\b[30] ), .out0(new_n354));
  aoi022aa1n02x7               g259(.a(new_n353), .b(new_n354), .c(new_n349), .d(new_n345), .o1(\s[31] ));
  inv000aa1d42x5               g260(.a(\b[1] ), .o1(new_n356));
  inv000aa1d42x5               g261(.a(\a[2] ), .o1(new_n357));
  aoi022aa1n02x5               g262(.a(new_n356), .b(new_n357), .c(\a[1] ), .d(\b[0] ), .o1(new_n358));
  oaib12aa1n02x5               g263(.a(new_n358), .b(new_n356), .c(\a[2] ), .out0(new_n359));
  aboi22aa1n03x5               g264(.a(new_n103), .b(new_n104), .c(new_n357), .d(new_n356), .out0(new_n360));
  aoi022aa1n02x5               g265(.a(new_n359), .b(new_n360), .c(new_n99), .d(new_n105), .o1(\s[3] ));
  aoai13aa1n02x5               g266(.a(new_n102), .b(new_n103), .c(new_n99), .d(new_n104), .o1(new_n362));
  aoi112aa1n02x5               g267(.a(new_n103), .b(new_n102), .c(new_n99), .d(new_n104), .o1(new_n363));
  norb02aa1n02x5               g268(.a(new_n362), .b(new_n363), .out0(\s[4] ));
  xnbna2aa1n03x5               g269(.a(new_n142), .b(new_n106), .c(new_n107), .out0(\s[5] ));
  aoai13aa1n02x5               g270(.a(new_n141), .b(new_n111), .c(new_n140), .d(new_n110), .o1(new_n366));
  aoi112aa1n02x5               g271(.a(new_n111), .b(new_n141), .c(new_n140), .d(new_n142), .o1(new_n367));
  norb02aa1n02x5               g272(.a(new_n366), .b(new_n367), .out0(\s[6] ));
  inv000aa1d42x5               g273(.a(new_n108), .o1(new_n369));
  xnbna2aa1n03x5               g274(.a(new_n116), .b(new_n366), .c(new_n369), .out0(\s[7] ));
  aob012aa1n02x5               g275(.a(new_n116), .b(new_n366), .c(new_n369), .out0(new_n371));
  oai012aa1n02x5               g276(.a(new_n371), .b(\b[6] ), .c(\a[7] ), .o1(new_n372));
  norp02aa1n02x5               g277(.a(new_n113), .b(new_n115), .o1(new_n373));
  aoi022aa1n02x5               g278(.a(new_n372), .b(new_n113), .c(new_n371), .d(new_n373), .o1(\s[8] ));
  nanp02aa1n02x5               g279(.a(new_n140), .b(new_n144), .o1(new_n375));
  aoi113aa1n02x5               g280(.a(new_n124), .b(new_n128), .c(new_n113), .d(new_n118), .e(new_n120), .o1(new_n376));
  aoi022aa1n02x5               g281(.a(new_n126), .b(new_n128), .c(new_n375), .d(new_n376), .o1(\s[9] ));
endmodule


