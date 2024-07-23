// Benchmark "adder" written by ABC on Wed Jul 17 22:09:57 2024

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
    new_n139, new_n140, new_n141, new_n142, new_n143, new_n144, new_n146,
    new_n147, new_n148, new_n149, new_n150, new_n152, new_n153, new_n154,
    new_n155, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n343, new_n345,
    new_n348, new_n349, new_n350, new_n352, new_n354, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[10] ), .o1(new_n97));
  inv040aa1d28x5               g002(.a(\b[9] ), .o1(new_n98));
  nanp02aa1n24x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nand42aa1d28x5               g004(.a(\b[9] ), .b(\a[10] ), .o1(new_n100));
  nand02aa1d04x5               g005(.a(new_n99), .b(new_n100), .o1(new_n101));
  nor002aa1n02x5               g006(.a(\b[8] ), .b(\a[9] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\a[3] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(\b[2] ), .o1(new_n104));
  nor022aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nand42aa1n16x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  aoi013aa1n06x4               g011(.a(new_n105), .b(new_n106), .c(new_n103), .d(new_n104), .o1(new_n107));
  nand22aa1n12x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  norp02aa1n24x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  nona22aa1n03x5               g015(.a(new_n109), .b(new_n110), .c(new_n108), .out0(new_n111));
  aoi022aa1n02x7               g016(.a(new_n104), .b(new_n103), .c(\a[2] ), .d(\b[1] ), .o1(new_n112));
  nand42aa1n06x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nano22aa1n03x7               g018(.a(new_n105), .b(new_n106), .c(new_n113), .out0(new_n114));
  nanp03aa1n06x5               g019(.a(new_n114), .b(new_n111), .c(new_n112), .o1(new_n115));
  nor042aa1d18x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand42aa1d28x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  norb02aa1n15x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  nor002aa1d32x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nand02aa1n08x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  norb02aa1n06x5               g025(.a(new_n120), .b(new_n119), .out0(new_n121));
  nand42aa1d28x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nor002aa1d32x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nor042aa1n04x5               g028(.a(\b[7] ), .b(\a[8] ), .o1(new_n124));
  nand02aa1n12x5               g029(.a(\b[7] ), .b(\a[8] ), .o1(new_n125));
  nano23aa1n09x5               g030(.a(new_n124), .b(new_n123), .c(new_n125), .d(new_n122), .out0(new_n126));
  nand03aa1n02x5               g031(.a(new_n126), .b(new_n118), .c(new_n121), .o1(new_n127));
  aoi022aa1n06x5               g032(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n128));
  inv030aa1n02x5               g033(.a(new_n116), .o1(new_n129));
  tech160nm_fioai012aa1n05x5   g034(.a(new_n122), .b(new_n123), .c(new_n119), .o1(new_n130));
  nanp02aa1n03x5               g035(.a(new_n130), .b(new_n129), .o1(new_n131));
  tech160nm_fiaoi012aa1n03p5x5 g036(.a(new_n124), .b(new_n131), .c(new_n128), .o1(new_n132));
  aoai13aa1n12x5               g037(.a(new_n132), .b(new_n127), .c(new_n115), .d(new_n107), .o1(new_n133));
  xorc02aa1n12x5               g038(.a(\a[9] ), .b(\b[8] ), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n101), .b(new_n102), .c(new_n133), .d(new_n134), .o1(new_n135));
  oai122aa1n03x5               g040(.a(new_n109), .b(new_n110), .c(new_n108), .d(\b[2] ), .e(\a[3] ), .o1(new_n136));
  nanb03aa1n03x5               g041(.a(new_n105), .b(new_n113), .c(new_n106), .out0(new_n137));
  oai012aa1n06x5               g042(.a(new_n107), .b(new_n136), .c(new_n137), .o1(new_n138));
  nona23aa1n09x5               g043(.a(new_n122), .b(new_n125), .c(new_n124), .d(new_n123), .out0(new_n139));
  nano22aa1n03x7               g044(.a(new_n139), .b(new_n118), .c(new_n121), .out0(new_n140));
  oaoi13aa1n06x5               g045(.a(new_n116), .b(new_n122), .c(new_n119), .d(new_n123), .o1(new_n141));
  obai22aa1n09x5               g046(.a(new_n128), .b(new_n141), .c(\a[8] ), .d(\b[7] ), .out0(new_n142));
  aoai13aa1n03x5               g047(.a(new_n134), .b(new_n142), .c(new_n138), .d(new_n140), .o1(new_n143));
  nona22aa1n02x4               g048(.a(new_n143), .b(new_n102), .c(new_n101), .out0(new_n144));
  nanp02aa1n02x5               g049(.a(new_n135), .b(new_n144), .o1(\s[10] ));
  xnrc02aa1n12x5               g050(.a(\b[10] ), .b(\a[11] ), .out0(new_n146));
  oai112aa1n06x5               g051(.a(new_n99), .b(new_n100), .c(\b[8] ), .d(\a[9] ), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n100), .b(new_n147), .c(new_n133), .d(new_n134), .o1(new_n148));
  aoi022aa1d24x5               g053(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n149));
  oa0012aa1n02x5               g054(.a(new_n149), .b(\b[10] ), .c(\a[11] ), .o(new_n150));
  aoi022aa1n02x5               g055(.a(new_n148), .b(new_n146), .c(new_n144), .d(new_n150), .o1(\s[11] ));
  inv000aa1d42x5               g056(.a(\a[12] ), .o1(new_n152));
  inv000aa1d42x5               g057(.a(\a[11] ), .o1(new_n153));
  inv040aa1d32x5               g058(.a(\b[10] ), .o1(new_n154));
  aoi022aa1n03x5               g059(.a(new_n144), .b(new_n149), .c(new_n154), .d(new_n153), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[11] ), .c(new_n152), .out0(\s[12] ));
  xorc02aa1n12x5               g061(.a(\a[12] ), .b(\b[11] ), .out0(new_n157));
  nona23aa1d16x5               g062(.a(new_n134), .b(new_n157), .c(new_n146), .d(new_n101), .out0(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n142), .c(new_n138), .d(new_n140), .o1(new_n160));
  nand02aa1n08x5               g065(.a(\b[11] ), .b(\a[12] ), .o1(new_n161));
  aboi22aa1d24x5               g066(.a(\b[11] ), .b(new_n152), .c(new_n153), .d(new_n154), .out0(new_n162));
  inv040aa1n02x5               g067(.a(new_n162), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n161), .b(new_n163), .c(new_n147), .d(new_n149), .o1(new_n164));
  nor002aa1d32x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nand42aa1d28x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n06x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n160), .c(new_n164), .out0(\s[13] ));
  inv000aa1d42x5               g073(.a(\a[14] ), .o1(new_n169));
  aoi012aa1n12x5               g074(.a(new_n142), .b(new_n138), .c(new_n140), .o1(new_n170));
  tech160nm_fioai012aa1n05x5   g075(.a(new_n164), .b(new_n170), .c(new_n158), .o1(new_n171));
  aoi012aa1n03x5               g076(.a(new_n165), .b(new_n171), .c(new_n166), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[13] ), .c(new_n169), .out0(\s[14] ));
  tech160nm_fixnrc02aa1n04x5   g078(.a(\b[13] ), .b(\a[14] ), .out0(new_n174));
  norb02aa1n06x4               g079(.a(new_n167), .b(new_n174), .out0(new_n175));
  nanp02aa1n03x5               g080(.a(new_n171), .b(new_n175), .o1(new_n176));
  inv040aa1d28x5               g081(.a(\b[13] ), .o1(new_n177));
  oaoi03aa1n12x5               g082(.a(new_n169), .b(new_n177), .c(new_n165), .o1(new_n178));
  xorc02aa1n12x5               g083(.a(\a[15] ), .b(\b[14] ), .out0(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n176), .c(new_n178), .out0(\s[15] ));
  nanp02aa1n03x5               g085(.a(new_n176), .b(new_n178), .o1(new_n181));
  norp02aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  xnrc02aa1n12x5               g087(.a(\b[15] ), .b(\a[16] ), .out0(new_n183));
  aoai13aa1n03x5               g088(.a(new_n183), .b(new_n182), .c(new_n181), .d(new_n179), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n178), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n179), .b(new_n185), .c(new_n171), .d(new_n175), .o1(new_n186));
  nona22aa1n02x4               g091(.a(new_n186), .b(new_n183), .c(new_n182), .out0(new_n187));
  nanp02aa1n03x5               g092(.a(new_n184), .b(new_n187), .o1(\s[16] ));
  tech160nm_fixnrc02aa1n02p5x5 g093(.a(\b[14] ), .b(\a[15] ), .out0(new_n189));
  nor002aa1n03x5               g094(.a(new_n183), .b(new_n189), .o1(new_n190));
  nano22aa1n12x5               g095(.a(new_n158), .b(new_n175), .c(new_n190), .out0(new_n191));
  nand22aa1n03x5               g096(.a(new_n133), .b(new_n191), .o1(new_n192));
  norp02aa1n02x5               g097(.a(\b[9] ), .b(\a[10] ), .o1(new_n193));
  oai122aa1n03x5               g098(.a(new_n100), .b(new_n193), .c(new_n102), .d(new_n154), .e(new_n153), .o1(new_n194));
  and002aa1n02x5               g099(.a(\b[13] ), .b(\a[14] ), .o(new_n195));
  aoi022aa1n02x5               g100(.a(new_n177), .b(new_n169), .c(\a[13] ), .d(\b[12] ), .o1(new_n196));
  nona23aa1n03x5               g101(.a(new_n196), .b(new_n161), .c(new_n195), .d(new_n165), .out0(new_n197));
  aoai13aa1n06x5               g102(.a(new_n178), .b(new_n197), .c(new_n194), .d(new_n162), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n199));
  oab012aa1n02x4               g104(.a(new_n199), .b(\a[16] ), .c(\b[15] ), .out0(new_n200));
  aobi12aa1n06x5               g105(.a(new_n200), .b(new_n198), .c(new_n190), .out0(new_n201));
  xorc02aa1n12x5               g106(.a(\a[17] ), .b(\b[16] ), .out0(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n192), .c(new_n201), .out0(\s[17] ));
  inv040aa1d32x5               g108(.a(\a[17] ), .o1(new_n204));
  inv040aa1d28x5               g109(.a(\b[16] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(new_n205), .b(new_n204), .o1(new_n206));
  inv030aa1n02x5               g111(.a(new_n190), .o1(new_n207));
  oaih12aa1n02x5               g112(.a(new_n161), .b(\b[12] ), .c(\a[13] ), .o1(new_n208));
  oai012aa1n02x5               g113(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .o1(new_n209));
  nor043aa1n03x5               g114(.a(new_n209), .b(new_n208), .c(new_n195), .o1(new_n210));
  aoai13aa1n04x5               g115(.a(new_n210), .b(new_n163), .c(new_n147), .d(new_n149), .o1(new_n211));
  aoai13aa1n06x5               g116(.a(new_n200), .b(new_n207), .c(new_n211), .d(new_n178), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n202), .b(new_n212), .c(new_n133), .d(new_n191), .o1(new_n213));
  nor002aa1d32x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nand02aa1n10x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  nanb02aa1n12x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  xobna2aa1n03x5               g121(.a(new_n216), .b(new_n213), .c(new_n206), .out0(\s[18] ));
  norb02aa1n02x5               g122(.a(new_n202), .b(new_n216), .out0(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n212), .c(new_n133), .d(new_n191), .o1(new_n219));
  aoai13aa1n12x5               g124(.a(new_n215), .b(new_n214), .c(new_n204), .d(new_n205), .o1(new_n220));
  nor002aa1d32x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nand02aa1d06x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  norb02aa1n06x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n219), .c(new_n220), .out0(\s[19] ));
  xnrc02aa1n02x5               g129(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n04x5               g130(.a(new_n219), .b(new_n220), .o1(new_n226));
  nor002aa1d32x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nand22aa1n12x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  nanb02aa1n06x5               g133(.a(new_n227), .b(new_n228), .out0(new_n229));
  aoai13aa1n03x5               g134(.a(new_n229), .b(new_n221), .c(new_n226), .d(new_n223), .o1(new_n230));
  nano23aa1n09x5               g135(.a(new_n174), .b(new_n183), .c(new_n179), .d(new_n167), .out0(new_n231));
  nanb02aa1n12x5               g136(.a(new_n158), .b(new_n231), .out0(new_n232));
  oai012aa1n06x5               g137(.a(new_n201), .b(new_n232), .c(new_n170), .o1(new_n233));
  oaoi03aa1n12x5               g138(.a(\a[18] ), .b(\b[17] ), .c(new_n206), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n223), .b(new_n234), .c(new_n233), .d(new_n218), .o1(new_n235));
  nona22aa1n03x5               g140(.a(new_n235), .b(new_n229), .c(new_n221), .out0(new_n236));
  nanp02aa1n03x5               g141(.a(new_n230), .b(new_n236), .o1(\s[20] ));
  nona23aa1n09x5               g142(.a(new_n228), .b(new_n222), .c(new_n221), .d(new_n227), .out0(new_n238));
  ao0012aa1n12x5               g143(.a(new_n227), .b(new_n221), .c(new_n228), .o(new_n239));
  oabi12aa1n18x5               g144(.a(new_n239), .b(new_n238), .c(new_n220), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  nona23aa1d18x5               g146(.a(new_n223), .b(new_n202), .c(new_n229), .d(new_n216), .out0(new_n242));
  aoai13aa1n02x7               g147(.a(new_n241), .b(new_n242), .c(new_n192), .d(new_n201), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[20] ), .b(\a[21] ), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  inv040aa1n03x5               g150(.a(new_n242), .o1(new_n246));
  aoi012aa1n02x5               g151(.a(new_n245), .b(new_n233), .c(new_n246), .o1(new_n247));
  aoi022aa1n02x5               g152(.a(new_n247), .b(new_n241), .c(new_n243), .d(new_n245), .o1(\s[21] ));
  nor042aa1n03x5               g153(.a(\b[20] ), .b(\a[21] ), .o1(new_n249));
  tech160nm_fixnrc02aa1n05x5   g154(.a(\b[21] ), .b(\a[22] ), .out0(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n249), .c(new_n243), .d(new_n245), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n245), .b(new_n240), .c(new_n233), .d(new_n246), .o1(new_n252));
  nona22aa1n03x5               g157(.a(new_n252), .b(new_n250), .c(new_n249), .out0(new_n253));
  nanp02aa1n03x5               g158(.a(new_n251), .b(new_n253), .o1(\s[22] ));
  nor042aa1n06x5               g159(.a(new_n250), .b(new_n244), .o1(new_n255));
  inv000aa1n02x5               g160(.a(new_n255), .o1(new_n256));
  nor002aa1n02x5               g161(.a(new_n242), .b(new_n256), .o1(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n212), .c(new_n133), .d(new_n191), .o1(new_n258));
  nano23aa1n06x5               g163(.a(new_n221), .b(new_n227), .c(new_n228), .d(new_n222), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n255), .b(new_n239), .c(new_n259), .d(new_n234), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\a[22] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(\b[21] ), .o1(new_n262));
  oao003aa1n02x5               g167(.a(new_n261), .b(new_n262), .c(new_n249), .carry(new_n263));
  inv000aa1n02x5               g168(.a(new_n263), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n260), .b(new_n264), .o1(new_n265));
  nanb02aa1n06x5               g170(.a(new_n265), .b(new_n258), .out0(new_n266));
  xorc02aa1n12x5               g171(.a(\a[23] ), .b(\b[22] ), .out0(new_n267));
  aoi112aa1n02x5               g172(.a(new_n267), .b(new_n263), .c(new_n240), .d(new_n255), .o1(new_n268));
  aoi022aa1n02x5               g173(.a(new_n266), .b(new_n267), .c(new_n258), .d(new_n268), .o1(\s[23] ));
  norp02aa1n02x5               g174(.a(\b[22] ), .b(\a[23] ), .o1(new_n270));
  xnrc02aa1n12x5               g175(.a(\b[23] ), .b(\a[24] ), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n266), .d(new_n267), .o1(new_n272));
  aoai13aa1n03x5               g177(.a(new_n267), .b(new_n265), .c(new_n233), .d(new_n257), .o1(new_n273));
  nona22aa1n03x5               g178(.a(new_n273), .b(new_n271), .c(new_n270), .out0(new_n274));
  nanp02aa1n03x5               g179(.a(new_n272), .b(new_n274), .o1(\s[24] ));
  nanb02aa1n02x5               g180(.a(new_n271), .b(new_n267), .out0(new_n276));
  nor043aa1n02x5               g181(.a(new_n242), .b(new_n256), .c(new_n276), .o1(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n212), .c(new_n133), .d(new_n191), .o1(new_n278));
  aoi112aa1n02x5               g183(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n279));
  oab012aa1n02x4               g184(.a(new_n279), .b(\a[24] ), .c(\b[23] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n276), .c(new_n260), .d(new_n264), .o1(new_n281));
  nanb02aa1n03x5               g186(.a(new_n281), .b(new_n278), .out0(new_n282));
  xorb03aa1n02x5               g187(.a(new_n282), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  xorc02aa1n12x5               g189(.a(\a[25] ), .b(\b[24] ), .out0(new_n285));
  xnrc02aa1n12x5               g190(.a(\b[25] ), .b(\a[26] ), .out0(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n284), .c(new_n282), .d(new_n285), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n285), .b(new_n281), .c(new_n233), .d(new_n277), .o1(new_n288));
  nona22aa1n03x5               g193(.a(new_n288), .b(new_n286), .c(new_n284), .out0(new_n289));
  nanp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[26] ));
  norb02aa1n12x5               g195(.a(new_n285), .b(new_n286), .out0(new_n291));
  nano23aa1n03x7               g196(.a(new_n242), .b(new_n276), .c(new_n291), .d(new_n255), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n212), .c(new_n133), .d(new_n191), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(new_n281), .b(new_n291), .o1(new_n294));
  aoi112aa1n02x5               g199(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n295));
  oab012aa1n02x4               g200(.a(new_n295), .b(\a[26] ), .c(\b[25] ), .out0(new_n296));
  nand43aa1n02x5               g201(.a(new_n293), .b(new_n294), .c(new_n296), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  nano22aa1n02x4               g203(.a(new_n298), .b(new_n294), .c(new_n296), .out0(new_n299));
  aoi022aa1n02x7               g204(.a(new_n299), .b(new_n293), .c(new_n297), .d(new_n298), .o1(\s[27] ));
  norp02aa1n02x5               g205(.a(\b[26] ), .b(\a[27] ), .o1(new_n301));
  norp02aa1n02x5               g206(.a(\b[27] ), .b(\a[28] ), .o1(new_n302));
  nand42aa1n03x5               g207(.a(\b[27] ), .b(\a[28] ), .o1(new_n303));
  nanb02aa1n06x5               g208(.a(new_n302), .b(new_n303), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n301), .c(new_n297), .d(new_n298), .o1(new_n305));
  nona23aa1d18x5               g210(.a(new_n246), .b(new_n291), .c(new_n276), .d(new_n256), .out0(new_n306));
  oaoi13aa1n09x5               g211(.a(new_n306), .b(new_n201), .c(new_n232), .d(new_n170), .o1(new_n307));
  norb02aa1n02x5               g212(.a(new_n267), .b(new_n271), .out0(new_n308));
  aoai13aa1n04x5               g213(.a(new_n308), .b(new_n263), .c(new_n240), .d(new_n255), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n291), .o1(new_n310));
  aoai13aa1n06x5               g215(.a(new_n296), .b(new_n310), .c(new_n309), .d(new_n280), .o1(new_n311));
  oaih12aa1n02x5               g216(.a(new_n298), .b(new_n311), .c(new_n307), .o1(new_n312));
  nona22aa1n03x5               g217(.a(new_n312), .b(new_n304), .c(new_n301), .out0(new_n313));
  nanp02aa1n03x5               g218(.a(new_n305), .b(new_n313), .o1(\s[28] ));
  norb02aa1n03x5               g219(.a(new_n298), .b(new_n304), .out0(new_n315));
  oaih12aa1n02x5               g220(.a(new_n315), .b(new_n311), .c(new_n307), .o1(new_n316));
  aobi12aa1n02x7               g221(.a(new_n296), .b(new_n281), .c(new_n291), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n315), .o1(new_n318));
  oai012aa1n02x5               g223(.a(new_n303), .b(new_n302), .c(new_n301), .o1(new_n319));
  aoai13aa1n02x7               g224(.a(new_n319), .b(new_n318), .c(new_n317), .d(new_n293), .o1(new_n320));
  norp02aa1n02x5               g225(.a(\b[28] ), .b(\a[29] ), .o1(new_n321));
  nand42aa1n03x5               g226(.a(\b[28] ), .b(\a[29] ), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(new_n323));
  oai022aa1n02x5               g228(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n324));
  aboi22aa1n03x5               g229(.a(new_n321), .b(new_n322), .c(new_n324), .d(new_n303), .out0(new_n325));
  aoi022aa1n03x5               g230(.a(new_n320), .b(new_n323), .c(new_n316), .d(new_n325), .o1(\s[29] ));
  xorb03aa1n02x5               g231(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g232(.a(new_n304), .b(new_n298), .c(new_n323), .out0(new_n328));
  oaih12aa1n02x5               g233(.a(new_n328), .b(new_n311), .c(new_n307), .o1(new_n329));
  inv000aa1n02x5               g234(.a(new_n328), .o1(new_n330));
  aoi013aa1n02x4               g235(.a(new_n321), .b(new_n324), .c(new_n303), .d(new_n322), .o1(new_n331));
  aoai13aa1n02x7               g236(.a(new_n331), .b(new_n330), .c(new_n317), .d(new_n293), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .out0(new_n333));
  aoi113aa1n02x5               g238(.a(new_n333), .b(new_n321), .c(new_n324), .d(new_n322), .e(new_n303), .o1(new_n334));
  aoi022aa1n03x5               g239(.a(new_n332), .b(new_n333), .c(new_n329), .d(new_n334), .o1(\s[30] ));
  nand03aa1n02x5               g240(.a(new_n315), .b(new_n323), .c(new_n333), .o1(new_n336));
  oabi12aa1n03x5               g241(.a(new_n336), .b(new_n311), .c(new_n307), .out0(new_n337));
  oao003aa1n02x5               g242(.a(\a[30] ), .b(\b[29] ), .c(new_n331), .carry(new_n338));
  aoai13aa1n02x7               g243(.a(new_n338), .b(new_n336), .c(new_n317), .d(new_n293), .o1(new_n339));
  xorc02aa1n02x5               g244(.a(\a[31] ), .b(\b[30] ), .out0(new_n340));
  norb02aa1n02x5               g245(.a(new_n338), .b(new_n340), .out0(new_n341));
  aoi022aa1n03x5               g246(.a(new_n339), .b(new_n340), .c(new_n337), .d(new_n341), .o1(\s[31] ));
  oai012aa1n02x5               g247(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n343));
  xorb03aa1n02x5               g248(.a(new_n343), .b(\b[2] ), .c(new_n103), .out0(\s[3] ));
  oaoi03aa1n02x5               g249(.a(\a[3] ), .b(\b[2] ), .c(new_n343), .o1(new_n345));
  xorb03aa1n02x5               g250(.a(new_n345), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g251(.a(new_n121), .b(new_n115), .c(new_n107), .out0(\s[5] ));
  norb02aa1n02x5               g252(.a(new_n122), .b(new_n123), .out0(new_n348));
  aoai13aa1n02x5               g253(.a(new_n348), .b(new_n119), .c(new_n138), .d(new_n120), .o1(new_n349));
  aoi112aa1n02x5               g254(.a(new_n119), .b(new_n348), .c(new_n138), .d(new_n120), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n349), .b(new_n350), .out0(\s[6] ));
  inv000aa1d42x5               g256(.a(new_n123), .o1(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n118), .b(new_n349), .c(new_n352), .out0(\s[7] ));
  inv000aa1d42x5               g258(.a(new_n118), .o1(new_n354));
  aoai13aa1n02x5               g259(.a(new_n129), .b(new_n354), .c(new_n349), .d(new_n352), .o1(new_n355));
  xorb03aa1n02x5               g260(.a(new_n355), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g261(.a(new_n170), .b(new_n134), .out0(\s[9] ));
endmodule

