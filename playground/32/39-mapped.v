// Benchmark "adder" written by ABC on Thu Jul 18 04:42:53 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n323, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n348, new_n349, new_n351, new_n352,
    new_n354, new_n355, new_n357, new_n358, new_n360, new_n361, new_n362,
    new_n363, new_n364, new_n366, new_n368, new_n369;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor022aa1n06x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  aoi022aa1d24x5               g003(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n99));
  nor042aa1n04x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nand42aa1n08x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  norb02aa1n12x5               g006(.a(new_n101), .b(new_n100), .out0(new_n102));
  oai022aa1d18x5               g007(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n103));
  oaoi13aa1n12x5               g008(.a(new_n103), .b(new_n102), .c(new_n99), .d(new_n98), .o1(new_n104));
  inv020aa1d32x5               g009(.a(\a[8] ), .o1(new_n105));
  inv040aa1d28x5               g010(.a(\b[7] ), .o1(new_n106));
  norp02aa1n12x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  and002aa1n12x5               g012(.a(\b[6] ), .b(\a[7] ), .o(new_n108));
  aoi112aa1n09x5               g013(.a(new_n108), .b(new_n107), .c(new_n105), .d(new_n106), .o1(new_n109));
  nor002aa1n06x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  tech160nm_fiaoi012aa1n03p5x5 g015(.a(new_n110), .b(\a[8] ), .c(\b[7] ), .o1(new_n111));
  nand02aa1n03x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  tech160nm_finand02aa1n03p5x5 g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  tech160nm_finand02aa1n03p5x5 g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  oai012aa1n04x7               g019(.a(new_n114), .b(\b[4] ), .c(\a[5] ), .o1(new_n115));
  nano22aa1n03x7               g020(.a(new_n115), .b(new_n112), .c(new_n113), .out0(new_n116));
  nand23aa1n06x5               g021(.a(new_n116), .b(new_n109), .c(new_n111), .o1(new_n117));
  norp02aa1n04x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  aoi022aa1n06x5               g023(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n119));
  oa0012aa1n06x5               g024(.a(new_n119), .b(new_n118), .c(new_n110), .o(new_n120));
  oao003aa1n02x5               g025(.a(new_n105), .b(new_n106), .c(new_n107), .carry(new_n121));
  tech160nm_fiaoi012aa1n05x5   g026(.a(new_n121), .b(new_n120), .c(new_n109), .o1(new_n122));
  oai012aa1d24x5               g027(.a(new_n122), .b(new_n117), .c(new_n104), .o1(new_n123));
  nand42aa1n16x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n97), .b(new_n123), .c(new_n124), .o1(new_n125));
  nor042aa1n03x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand02aa1n06x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n02x7               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  norb02aa1n03x5               g033(.a(new_n124), .b(new_n97), .out0(new_n129));
  norb03aa1n02x5               g034(.a(new_n127), .b(new_n97), .c(new_n126), .out0(new_n130));
  aob012aa1n02x5               g035(.a(new_n130), .b(new_n123), .c(new_n129), .out0(new_n131));
  oai012aa1n02x5               g036(.a(new_n131), .b(new_n125), .c(new_n128), .o1(\s[10] ));
  oai012aa1n02x5               g037(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n133));
  nona23aa1n02x4               g038(.a(new_n127), .b(new_n124), .c(new_n97), .d(new_n126), .out0(new_n134));
  nanb02aa1n06x5               g039(.a(new_n134), .b(new_n123), .out0(new_n135));
  nor042aa1d18x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  tech160nm_finand02aa1n03p5x5 g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n135), .c(new_n133), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(new_n136), .o1(new_n140));
  aob012aa1n03x5               g045(.a(new_n138), .b(new_n135), .c(new_n133), .out0(new_n141));
  nor042aa1n03x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand02aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  nona23aa1n03x5               g049(.a(new_n141), .b(new_n143), .c(new_n142), .d(new_n136), .out0(new_n145));
  aoai13aa1n03x5               g050(.a(new_n145), .b(new_n144), .c(new_n141), .d(new_n140), .o1(\s[12] ));
  nano22aa1n02x5               g051(.a(new_n134), .b(new_n138), .c(new_n144), .out0(new_n147));
  nand02aa1d04x5               g052(.a(new_n123), .b(new_n147), .o1(new_n148));
  nano22aa1n03x7               g053(.a(new_n142), .b(new_n137), .c(new_n143), .out0(new_n149));
  tech160nm_fioai012aa1n04x5   g054(.a(new_n127), .b(\b[10] ), .c(\a[11] ), .o1(new_n150));
  oab012aa1n06x5               g055(.a(new_n150), .b(new_n97), .c(new_n126), .out0(new_n151));
  aoi012aa1n03x5               g056(.a(new_n142), .b(new_n136), .c(new_n143), .o1(new_n152));
  aobi12aa1n06x5               g057(.a(new_n152), .b(new_n151), .c(new_n149), .out0(new_n153));
  nand02aa1d04x5               g058(.a(new_n148), .b(new_n153), .o1(new_n154));
  nor002aa1d32x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1d28x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norb02aa1n03x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  nanp02aa1n06x5               g062(.a(new_n151), .b(new_n149), .o1(new_n158));
  nano22aa1n02x4               g063(.a(new_n157), .b(new_n158), .c(new_n152), .out0(new_n159));
  aoi022aa1n02x5               g064(.a(new_n154), .b(new_n157), .c(new_n148), .d(new_n159), .o1(\s[13] ));
  inv000aa1d42x5               g065(.a(new_n155), .o1(new_n161));
  nanp02aa1n04x5               g066(.a(new_n158), .b(new_n152), .o1(new_n162));
  aoai13aa1n02x7               g067(.a(new_n157), .b(new_n162), .c(new_n123), .d(new_n147), .o1(new_n163));
  nor002aa1n06x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand42aa1d28x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nona23aa1n02x4               g071(.a(new_n163), .b(new_n165), .c(new_n164), .d(new_n155), .out0(new_n167));
  aoai13aa1n03x5               g072(.a(new_n167), .b(new_n166), .c(new_n161), .d(new_n163), .o1(\s[14] ));
  nano23aa1n06x5               g073(.a(new_n155), .b(new_n164), .c(new_n165), .d(new_n156), .out0(new_n169));
  oaoi03aa1n02x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n161), .o1(new_n170));
  tech160nm_fixorc02aa1n03p5x5 g075(.a(\a[15] ), .b(\b[14] ), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n170), .c(new_n154), .d(new_n169), .o1(new_n172));
  aoi112aa1n02x7               g077(.a(new_n171), .b(new_n170), .c(new_n154), .d(new_n169), .o1(new_n173));
  norb02aa1n03x4               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(\a[15] ), .o1(new_n175));
  inv000aa1d42x5               g080(.a(\b[14] ), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n175), .o1(new_n177));
  norp02aa1n12x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  and002aa1n12x5               g083(.a(\b[15] ), .b(\a[16] ), .o(new_n179));
  norp02aa1n06x5               g084(.a(new_n179), .b(new_n178), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n179), .b(new_n178), .c(new_n175), .d(new_n176), .o1(new_n181));
  nand02aa1d04x5               g086(.a(new_n172), .b(new_n181), .o1(new_n182));
  aoai13aa1n03x5               g087(.a(new_n182), .b(new_n180), .c(new_n177), .d(new_n172), .o1(\s[16] ));
  nano23aa1n03x5               g088(.a(new_n136), .b(new_n142), .c(new_n143), .d(new_n137), .out0(new_n184));
  nand03aa1n02x5               g089(.a(new_n169), .b(new_n171), .c(new_n180), .o1(new_n185));
  nano32aa1n03x7               g090(.a(new_n185), .b(new_n184), .c(new_n128), .d(new_n129), .out0(new_n186));
  nand02aa1d10x5               g091(.a(new_n123), .b(new_n186), .o1(new_n187));
  xnrc02aa1n02x5               g092(.a(\b[14] ), .b(\a[15] ), .out0(new_n188));
  nano32aa1n03x7               g093(.a(new_n188), .b(new_n180), .c(new_n157), .d(new_n166), .out0(new_n189));
  inv000aa1d42x5               g094(.a(new_n178), .o1(new_n190));
  aoi022aa1d24x5               g095(.a(\b[14] ), .b(\a[15] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n191));
  oai012aa1n06x5               g096(.a(new_n191), .b(new_n164), .c(new_n155), .o1(new_n192));
  aoai13aa1n03x5               g097(.a(new_n190), .b(new_n179), .c(new_n192), .d(new_n177), .o1(new_n193));
  aoi012aa1n12x5               g098(.a(new_n193), .b(new_n162), .c(new_n189), .o1(new_n194));
  nand02aa1d16x5               g099(.a(new_n187), .b(new_n194), .o1(new_n195));
  xorc02aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  aoi022aa1n02x5               g101(.a(new_n192), .b(new_n177), .c(\a[16] ), .d(\b[15] ), .o1(new_n197));
  nanb02aa1n02x5               g102(.a(new_n196), .b(new_n190), .out0(new_n198));
  aoi112aa1n02x5               g103(.a(new_n198), .b(new_n197), .c(new_n162), .d(new_n189), .o1(new_n199));
  aoi022aa1n02x5               g104(.a(new_n195), .b(new_n196), .c(new_n187), .d(new_n199), .o1(\s[17] ));
  nor002aa1d32x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  oaib12aa1n06x5               g108(.a(new_n195), .b(new_n203), .c(\b[16] ), .out0(new_n204));
  nor002aa1d32x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand42aa1d28x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n204), .c(new_n202), .out0(\s[18] ));
  nanp02aa1n02x5               g113(.a(\b[16] ), .b(\a[17] ), .o1(new_n209));
  nano23aa1n06x5               g114(.a(new_n201), .b(new_n205), .c(new_n206), .d(new_n209), .out0(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n202), .o1(new_n211));
  nor042aa1d18x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand42aa1n16x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  aoai13aa1n06x5               g119(.a(new_n214), .b(new_n211), .c(new_n195), .d(new_n210), .o1(new_n215));
  aoi112aa1n02x7               g120(.a(new_n214), .b(new_n211), .c(new_n195), .d(new_n210), .o1(new_n216));
  norb02aa1n03x4               g121(.a(new_n215), .b(new_n216), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv030aa1n03x5               g123(.a(new_n212), .o1(new_n219));
  nor002aa1d24x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nand02aa1d28x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  norb03aa1n02x5               g127(.a(new_n221), .b(new_n212), .c(new_n220), .out0(new_n223));
  nanp02aa1n03x5               g128(.a(new_n215), .b(new_n223), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n222), .c(new_n215), .d(new_n219), .o1(\s[20] ));
  nano23aa1n02x4               g130(.a(new_n212), .b(new_n220), .c(new_n221), .d(new_n213), .out0(new_n226));
  nand42aa1n02x5               g131(.a(new_n226), .b(new_n210), .o1(new_n227));
  nanb02aa1n02x5               g132(.a(new_n227), .b(new_n195), .out0(new_n228));
  nanb03aa1n06x5               g133(.a(new_n220), .b(new_n221), .c(new_n213), .out0(new_n229));
  oai112aa1n06x5               g134(.a(new_n219), .b(new_n206), .c(new_n205), .d(new_n201), .o1(new_n230));
  aoi012aa1d24x5               g135(.a(new_n220), .b(new_n212), .c(new_n221), .o1(new_n231));
  oai012aa1n18x5               g136(.a(new_n231), .b(new_n230), .c(new_n229), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n227), .c(new_n187), .d(new_n194), .o1(new_n234));
  nor002aa1d32x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  nand42aa1n20x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  nano22aa1n03x7               g142(.a(new_n220), .b(new_n213), .c(new_n221), .out0(new_n238));
  tech160nm_fioai012aa1n03p5x5 g143(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .o1(new_n239));
  oab012aa1n06x5               g144(.a(new_n239), .b(new_n201), .c(new_n205), .out0(new_n240));
  inv020aa1n03x5               g145(.a(new_n231), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(new_n241), .b(new_n237), .c(new_n240), .d(new_n238), .o1(new_n242));
  aoi022aa1n02x5               g147(.a(new_n242), .b(new_n228), .c(new_n234), .d(new_n237), .o1(\s[21] ));
  nor002aa1n20x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nand42aa1n20x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  tech160nm_fiaoi012aa1n05x5   g151(.a(new_n235), .b(new_n234), .c(new_n236), .o1(new_n247));
  norb03aa1n02x5               g152(.a(new_n245), .b(new_n235), .c(new_n244), .out0(new_n248));
  aob012aa1n03x5               g153(.a(new_n248), .b(new_n234), .c(new_n237), .out0(new_n249));
  oaih12aa1n02x5               g154(.a(new_n249), .b(new_n247), .c(new_n246), .o1(\s[22] ));
  nano23aa1n09x5               g155(.a(new_n235), .b(new_n244), .c(new_n245), .d(new_n236), .out0(new_n251));
  inv000aa1n02x5               g156(.a(new_n251), .o1(new_n252));
  nano22aa1n03x7               g157(.a(new_n252), .b(new_n210), .c(new_n226), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n251), .b(new_n241), .c(new_n240), .d(new_n238), .o1(new_n254));
  oa0012aa1n02x5               g159(.a(new_n245), .b(new_n244), .c(new_n235), .o(new_n255));
  inv030aa1n02x5               g160(.a(new_n255), .o1(new_n256));
  nanp02aa1n02x5               g161(.a(new_n254), .b(new_n256), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n257), .c(new_n195), .d(new_n253), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(new_n258), .b(new_n255), .c(new_n232), .d(new_n251), .o1(new_n260));
  aobi12aa1n02x5               g165(.a(new_n260), .b(new_n195), .c(new_n253), .out0(new_n261));
  norb02aa1n03x4               g166(.a(new_n259), .b(new_n261), .out0(\s[23] ));
  nor042aa1n06x5               g167(.a(\b[22] ), .b(\a[23] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  nor042aa1n09x5               g169(.a(\b[23] ), .b(\a[24] ), .o1(new_n265));
  and002aa1n12x5               g170(.a(\b[23] ), .b(\a[24] ), .o(new_n266));
  nor042aa1n06x5               g171(.a(new_n266), .b(new_n265), .o1(new_n267));
  norp03aa1n02x5               g172(.a(new_n266), .b(new_n265), .c(new_n263), .o1(new_n268));
  nanp02aa1n03x5               g173(.a(new_n259), .b(new_n268), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n267), .c(new_n264), .d(new_n259), .o1(\s[24] ));
  and002aa1n06x5               g175(.a(new_n258), .b(new_n267), .o(new_n271));
  inv030aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  oab012aa1d18x5               g177(.a(new_n265), .b(new_n264), .c(new_n266), .out0(new_n273));
  aoai13aa1n12x5               g178(.a(new_n273), .b(new_n272), .c(new_n254), .d(new_n256), .o1(new_n274));
  oabi12aa1n02x5               g179(.a(new_n193), .b(new_n153), .c(new_n185), .out0(new_n275));
  tech160nm_fiaoi012aa1n05x5   g180(.a(new_n275), .b(new_n123), .c(new_n186), .o1(new_n276));
  nano22aa1n03x7               g181(.a(new_n276), .b(new_n253), .c(new_n271), .out0(new_n277));
  xorc02aa1n12x5               g182(.a(\a[25] ), .b(\b[24] ), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n271), .b(new_n255), .c(new_n232), .d(new_n251), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n278), .o1(new_n280));
  nano32aa1n02x4               g185(.a(new_n277), .b(new_n280), .c(new_n279), .d(new_n273), .out0(new_n281));
  oaoi13aa1n02x5               g186(.a(new_n281), .b(new_n278), .c(new_n274), .d(new_n277), .o1(\s[25] ));
  norp02aa1n02x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  oaih12aa1n02x5               g189(.a(new_n278), .b(new_n277), .c(new_n274), .o1(new_n285));
  tech160nm_fixorc02aa1n02p5x5 g190(.a(\a[26] ), .b(\b[25] ), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n274), .o1(new_n287));
  nona32aa1n06x5               g192(.a(new_n195), .b(new_n272), .c(new_n252), .d(new_n227), .out0(new_n288));
  nanp02aa1n02x5               g193(.a(\b[25] ), .b(\a[26] ), .o1(new_n289));
  oai022aa1n02x5               g194(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n290));
  norb02aa1n02x5               g195(.a(new_n289), .b(new_n290), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n280), .c(new_n288), .d(new_n287), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n286), .c(new_n285), .d(new_n284), .o1(\s[26] ));
  and002aa1n12x5               g198(.a(new_n286), .b(new_n278), .o(new_n294));
  nano23aa1n06x5               g199(.a(new_n272), .b(new_n227), .c(new_n294), .d(new_n251), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n275), .c(new_n123), .d(new_n186), .o1(new_n296));
  aoi022aa1n06x5               g201(.a(new_n274), .b(new_n294), .c(new_n289), .d(new_n290), .o1(new_n297));
  nand42aa1n02x5               g202(.a(new_n297), .b(new_n296), .o1(new_n298));
  nor042aa1d18x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  and002aa1n12x5               g204(.a(\b[26] ), .b(\a[27] ), .o(new_n300));
  nor042aa1n06x5               g205(.a(new_n300), .b(new_n299), .o1(new_n301));
  aoi122aa1n02x7               g206(.a(new_n301), .b(new_n289), .c(new_n290), .d(new_n274), .e(new_n294), .o1(new_n302));
  aoi022aa1n02x7               g207(.a(new_n298), .b(new_n301), .c(new_n302), .d(new_n296), .o1(\s[27] ));
  inv000aa1d42x5               g208(.a(new_n294), .o1(new_n304));
  nanp02aa1n02x5               g209(.a(new_n290), .b(new_n289), .o1(new_n305));
  aoai13aa1n04x5               g210(.a(new_n305), .b(new_n304), .c(new_n279), .d(new_n273), .o1(new_n306));
  nanp02aa1n02x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n306), .c(new_n195), .d(new_n295), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n299), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n300), .c(new_n297), .d(new_n296), .o1(new_n310));
  xorc02aa1n12x5               g215(.a(\a[28] ), .b(\b[27] ), .out0(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n299), .o1(new_n312));
  aoi022aa1n03x5               g217(.a(new_n310), .b(new_n311), .c(new_n308), .d(new_n312), .o1(\s[28] ));
  inv020aa1n03x5               g218(.a(new_n301), .o1(new_n314));
  norb02aa1n09x5               g219(.a(new_n311), .b(new_n314), .out0(new_n315));
  aoai13aa1n02x5               g220(.a(new_n315), .b(new_n306), .c(new_n195), .d(new_n295), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n315), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[28] ), .b(\b[27] ), .c(new_n309), .carry(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n317), .c(new_n297), .d(new_n296), .o1(new_n319));
  tech160nm_fixorc02aa1n03p5x5 g224(.a(\a[29] ), .b(\b[28] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n318), .b(new_n320), .out0(new_n321));
  aoi022aa1n03x5               g226(.a(new_n319), .b(new_n320), .c(new_n316), .d(new_n321), .o1(\s[29] ));
  nanp02aa1n02x5               g227(.a(\b[0] ), .b(\a[1] ), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g229(.a(new_n311), .b(new_n320), .c(new_n301), .o(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n306), .c(new_n195), .d(new_n295), .o1(new_n326));
  inv000aa1d42x5               g231(.a(new_n325), .o1(new_n327));
  inv000aa1d42x5               g232(.a(\b[28] ), .o1(new_n328));
  inv000aa1d42x5               g233(.a(\a[29] ), .o1(new_n329));
  oaib12aa1n02x5               g234(.a(new_n318), .b(\b[28] ), .c(new_n329), .out0(new_n330));
  oaib12aa1n02x5               g235(.a(new_n330), .b(new_n328), .c(\a[29] ), .out0(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n327), .c(new_n297), .d(new_n296), .o1(new_n332));
  tech160nm_fixorc02aa1n03p5x5 g237(.a(\a[30] ), .b(\b[29] ), .out0(new_n333));
  oaoi13aa1n02x5               g238(.a(new_n333), .b(new_n330), .c(new_n329), .d(new_n328), .o1(new_n334));
  aoi022aa1n03x5               g239(.a(new_n332), .b(new_n333), .c(new_n326), .d(new_n334), .o1(\s[30] ));
  nano32aa1n06x5               g240(.a(new_n314), .b(new_n333), .c(new_n311), .d(new_n320), .out0(new_n336));
  aoai13aa1n02x5               g241(.a(new_n336), .b(new_n306), .c(new_n195), .d(new_n295), .o1(new_n337));
  aoi022aa1n02x5               g242(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n338));
  norb02aa1n02x5               g243(.a(\a[31] ), .b(\b[30] ), .out0(new_n339));
  obai22aa1n02x7               g244(.a(\b[30] ), .b(\a[31] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n340));
  aoi112aa1n02x5               g245(.a(new_n340), .b(new_n339), .c(new_n330), .d(new_n338), .o1(new_n341));
  xorc02aa1n02x5               g246(.a(\a[31] ), .b(\b[30] ), .out0(new_n342));
  inv000aa1d42x5               g247(.a(new_n336), .o1(new_n343));
  norp02aa1n02x5               g248(.a(\b[29] ), .b(\a[30] ), .o1(new_n344));
  aoi012aa1n02x5               g249(.a(new_n344), .b(new_n330), .c(new_n338), .o1(new_n345));
  aoai13aa1n03x5               g250(.a(new_n345), .b(new_n343), .c(new_n297), .d(new_n296), .o1(new_n346));
  aoi022aa1n03x5               g251(.a(new_n346), .b(new_n342), .c(new_n337), .d(new_n341), .o1(\s[31] ));
  oai012aa1n02x5               g252(.a(new_n102), .b(new_n99), .c(new_n98), .o1(new_n348));
  norp03aa1n02x5               g253(.a(new_n102), .b(new_n99), .c(new_n98), .o1(new_n349));
  norb02aa1n02x5               g254(.a(new_n348), .b(new_n349), .out0(\s[3] ));
  xorc02aa1n02x5               g255(.a(\a[4] ), .b(\b[3] ), .out0(new_n351));
  norp02aa1n02x5               g256(.a(new_n351), .b(new_n100), .o1(new_n352));
  aboi22aa1n03x5               g257(.a(new_n104), .b(new_n351), .c(new_n352), .d(new_n348), .out0(\s[4] ));
  inv000aa1n06x5               g258(.a(new_n104), .o1(new_n354));
  nanb02aa1n02x5               g259(.a(new_n118), .b(new_n112), .out0(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n355), .b(new_n354), .c(new_n113), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g261(.a(new_n355), .b(new_n354), .c(new_n113), .o(new_n357));
  norb02aa1n02x5               g262(.a(new_n114), .b(new_n110), .out0(new_n358));
  xobna2aa1n03x5               g263(.a(new_n358), .b(new_n357), .c(new_n112), .out0(\s[6] ));
  xnrc02aa1n02x5               g264(.a(\b[6] ), .b(\a[7] ), .out0(new_n360));
  aoai13aa1n02x5               g265(.a(new_n112), .b(new_n118), .c(new_n354), .d(new_n113), .o1(new_n361));
  nanp02aa1n03x5               g266(.a(new_n361), .b(new_n358), .o1(new_n362));
  nanp02aa1n02x5               g267(.a(new_n362), .b(new_n114), .o1(new_n363));
  aoi112aa1n02x5               g268(.a(new_n108), .b(new_n107), .c(\a[6] ), .d(\b[5] ), .o1(new_n364));
  aoi022aa1n02x5               g269(.a(new_n363), .b(new_n360), .c(new_n362), .d(new_n364), .o1(\s[7] ));
  aoi012aa1n02x5               g270(.a(new_n107), .b(new_n362), .c(new_n364), .o1(new_n366));
  xorb03aa1n02x5               g271(.a(new_n366), .b(\b[7] ), .c(new_n105), .out0(\s[8] ));
  nano32aa1n02x4               g272(.a(new_n104), .b(new_n116), .c(new_n109), .d(new_n111), .out0(new_n368));
  aoi112aa1n02x5               g273(.a(new_n121), .b(new_n129), .c(new_n120), .d(new_n109), .o1(new_n369));
  aboi22aa1n03x5               g274(.a(new_n368), .b(new_n369), .c(new_n123), .d(new_n129), .out0(\s[9] ));
endmodule


