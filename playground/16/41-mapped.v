// Benchmark "adder" written by ABC on Wed Jul 17 20:32:11 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n332, new_n333, new_n334, new_n336, new_n337, new_n338, new_n339,
    new_n340, new_n342, new_n344, new_n345, new_n346, new_n348, new_n349,
    new_n350, new_n352, new_n353, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  orn002aa1n12x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nand02aa1n04x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aob012aa1n12x5               g005(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(new_n101));
  inv000aa1d42x5               g006(.a(\b[2] ), .o1(new_n102));
  nanb02aa1n12x5               g007(.a(\a[3] ), .b(new_n102), .out0(new_n103));
  nand42aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand02aa1d10x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  oa0022aa1n12x5               g010(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n106));
  aoai13aa1n12x5               g011(.a(new_n106), .b(new_n105), .c(new_n101), .d(new_n99), .o1(new_n107));
  inv000aa1d48x5               g012(.a(\a[5] ), .o1(new_n108));
  inv040aa1d30x5               g013(.a(\b[4] ), .o1(new_n109));
  tech160nm_finand02aa1n03p5x5 g014(.a(new_n109), .b(new_n108), .o1(new_n110));
  aoi022aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n111));
  norp02aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  aoi012aa1n02x5               g017(.a(new_n112), .b(\a[4] ), .c(\b[3] ), .o1(new_n113));
  aoi022aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n114));
  nor002aa1n16x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nona22aa1n02x4               g021(.a(new_n114), .b(new_n115), .c(new_n116), .out0(new_n117));
  nano32aa1n03x7               g022(.a(new_n117), .b(new_n113), .c(new_n111), .d(new_n110), .out0(new_n118));
  oaih22aa1n04x5               g023(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n119));
  inv040aa1d32x5               g024(.a(\a[7] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[6] ), .o1(new_n121));
  nand42aa1n08x5               g026(.a(\b[7] ), .b(\a[8] ), .o1(new_n122));
  aoai13aa1n06x5               g027(.a(new_n122), .b(new_n112), .c(new_n120), .d(new_n121), .o1(new_n123));
  nand02aa1n04x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  oai112aa1n04x5               g029(.a(new_n124), .b(new_n122), .c(new_n121), .d(new_n120), .o1(new_n125));
  tech160nm_fiaoi012aa1n04x5   g030(.a(new_n116), .b(new_n108), .c(new_n109), .o1(new_n126));
  oai013aa1n06x5               g031(.a(new_n123), .b(new_n125), .c(new_n126), .d(new_n119), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(new_n97), .b(new_n128), .out0(new_n129));
  inv000aa1n02x5               g034(.a(new_n129), .o1(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n127), .c(new_n118), .d(new_n107), .o1(new_n131));
  nor002aa1n04x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n98), .out0(\s[10] ));
  aob012aa1n03x5               g040(.a(new_n134), .b(new_n131), .c(new_n98), .out0(new_n136));
  oai012aa1n02x5               g041(.a(new_n133), .b(new_n132), .c(new_n97), .o1(new_n137));
  nor042aa1d18x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand22aa1n09x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1d21x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n136), .c(new_n137), .out0(\s[11] ));
  aob012aa1n02x5               g046(.a(new_n140), .b(new_n136), .c(new_n137), .out0(new_n142));
  nor042aa1n12x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n143), .o1(new_n146));
  aoi012aa1n02x5               g051(.a(new_n138), .b(new_n146), .c(new_n144), .o1(new_n147));
  inv000aa1n02x5               g052(.a(new_n138), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n140), .o1(new_n149));
  aoai13aa1n02x5               g054(.a(new_n148), .b(new_n149), .c(new_n136), .d(new_n137), .o1(new_n150));
  aboi22aa1n03x5               g055(.a(new_n145), .b(new_n150), .c(new_n142), .d(new_n147), .out0(\s[12] ));
  nona23aa1n02x4               g056(.a(new_n133), .b(new_n128), .c(new_n97), .d(new_n132), .out0(new_n152));
  nano32aa1n03x7               g057(.a(new_n152), .b(new_n140), .c(new_n146), .d(new_n144), .out0(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n127), .c(new_n118), .d(new_n107), .o1(new_n154));
  nanb03aa1n03x5               g059(.a(new_n143), .b(new_n144), .c(new_n139), .out0(new_n155));
  oai112aa1n03x5               g060(.a(new_n148), .b(new_n133), .c(new_n132), .d(new_n97), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n143), .b(new_n138), .c(new_n144), .o1(new_n157));
  oai012aa1n06x5               g062(.a(new_n157), .b(new_n156), .c(new_n155), .o1(new_n158));
  nanb02aa1n02x5               g063(.a(new_n158), .b(new_n154), .out0(new_n159));
  nor002aa1n06x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand02aa1d04x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  aoi112aa1n02x5               g067(.a(new_n162), .b(new_n143), .c(new_n144), .d(new_n138), .o1(new_n163));
  oa0012aa1n02x5               g068(.a(new_n163), .b(new_n156), .c(new_n155), .o(new_n164));
  aoi022aa1n02x5               g069(.a(new_n159), .b(new_n162), .c(new_n154), .d(new_n164), .o1(\s[13] ));
  orn002aa1n02x5               g070(.a(\a[13] ), .b(\b[12] ), .o(new_n166));
  nanp02aa1n02x5               g071(.a(new_n159), .b(new_n162), .o1(new_n167));
  nor002aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand02aa1d04x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n167), .c(new_n166), .out0(\s[14] ));
  nona23aa1d18x5               g076(.a(new_n169), .b(new_n161), .c(new_n160), .d(new_n168), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  oaoi03aa1n02x5               g078(.a(\a[14] ), .b(\b[13] ), .c(new_n166), .o1(new_n174));
  nor042aa1n03x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nand02aa1n03x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanb02aa1n12x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n174), .c(new_n159), .d(new_n173), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n178), .b(new_n174), .c(new_n159), .d(new_n173), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(\s[15] ));
  inv040aa1d32x5               g086(.a(\a[16] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[15] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n183), .b(new_n182), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  nanp02aa1n04x5               g090(.a(new_n184), .b(new_n185), .o1(new_n186));
  aoi012aa1n02x5               g091(.a(new_n175), .b(new_n184), .c(new_n185), .o1(new_n187));
  oai012aa1n02x5               g092(.a(new_n179), .b(\b[14] ), .c(\a[15] ), .o1(new_n188));
  aboi22aa1n03x5               g093(.a(new_n186), .b(new_n188), .c(new_n179), .d(new_n187), .out0(\s[16] ));
  aoi012aa1d18x5               g094(.a(new_n127), .b(new_n118), .c(new_n107), .o1(new_n190));
  nor043aa1d12x5               g095(.a(new_n172), .b(new_n177), .c(new_n186), .o1(new_n191));
  nand02aa1d04x5               g096(.a(new_n191), .b(new_n153), .o1(new_n192));
  orn002aa1n02x5               g097(.a(new_n190), .b(new_n192), .o(new_n193));
  nanp03aa1n02x5               g098(.a(new_n184), .b(new_n176), .c(new_n185), .o1(new_n194));
  oai122aa1n02x7               g099(.a(new_n169), .b(new_n168), .c(new_n160), .d(\b[14] ), .e(\a[15] ), .o1(new_n195));
  oaoi03aa1n02x5               g100(.a(new_n182), .b(new_n183), .c(new_n175), .o1(new_n196));
  oai012aa1n02x7               g101(.a(new_n196), .b(new_n195), .c(new_n194), .o1(new_n197));
  aoi012aa1n12x5               g102(.a(new_n197), .b(new_n191), .c(new_n158), .o1(new_n198));
  oai012aa1d24x5               g103(.a(new_n198), .b(new_n190), .c(new_n192), .o1(new_n199));
  xorc02aa1n02x5               g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  norp02aa1n02x5               g105(.a(new_n195), .b(new_n194), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n196), .out0(new_n202));
  aoi112aa1n02x5               g107(.a(new_n201), .b(new_n202), .c(new_n191), .d(new_n158), .o1(new_n203));
  aoi022aa1n02x5               g108(.a(new_n193), .b(new_n203), .c(new_n199), .d(new_n200), .o1(\s[17] ));
  inv000aa1d42x5               g109(.a(\a[17] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(\b[16] ), .b(new_n205), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n199), .b(new_n200), .o1(new_n207));
  xorc02aa1n02x5               g112(.a(\a[18] ), .b(\b[17] ), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n207), .c(new_n206), .out0(\s[18] ));
  inv040aa1d32x5               g114(.a(\a[18] ), .o1(new_n210));
  xroi22aa1d06x4               g115(.a(new_n205), .b(\b[16] ), .c(new_n210), .d(\b[17] ), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n199), .b(new_n211), .o1(new_n212));
  oaih22aa1n04x5               g117(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n213));
  oaib12aa1n02x5               g118(.a(new_n213), .b(new_n210), .c(\b[17] ), .out0(new_n214));
  xorc02aa1n12x5               g119(.a(\a[19] ), .b(\b[18] ), .out0(new_n215));
  xnbna2aa1n03x5               g120(.a(new_n215), .b(new_n212), .c(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n02x5               g122(.a(new_n215), .b(new_n212), .c(new_n214), .out0(new_n218));
  norp02aa1n12x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  and002aa1n12x5               g124(.a(\b[19] ), .b(\a[20] ), .o(new_n220));
  norp02aa1n04x5               g125(.a(new_n220), .b(new_n219), .o1(new_n221));
  nor022aa1n08x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  oab012aa1n02x4               g127(.a(new_n222), .b(new_n220), .c(new_n219), .out0(new_n223));
  aobi12aa1n02x5               g128(.a(new_n214), .b(new_n199), .c(new_n211), .out0(new_n224));
  oaoi03aa1n02x5               g129(.a(\a[19] ), .b(\b[18] ), .c(new_n224), .o1(new_n225));
  aoi022aa1n02x5               g130(.a(new_n225), .b(new_n221), .c(new_n218), .d(new_n223), .o1(\s[20] ));
  nand23aa1n06x5               g131(.a(new_n211), .b(new_n215), .c(new_n221), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoi112aa1n06x5               g133(.a(new_n220), .b(new_n219), .c(\a[19] ), .d(\b[18] ), .o1(new_n229));
  aoi012aa1n06x5               g134(.a(new_n222), .b(\a[18] ), .c(\b[17] ), .o1(new_n230));
  nand23aa1n06x5               g135(.a(new_n229), .b(new_n213), .c(new_n230), .o1(new_n231));
  aoib12aa1n12x5               g136(.a(new_n219), .b(new_n222), .c(new_n220), .out0(new_n232));
  nanp02aa1n02x5               g137(.a(new_n231), .b(new_n232), .o1(new_n233));
  xorc02aa1n12x5               g138(.a(\a[21] ), .b(\b[20] ), .out0(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n233), .c(new_n199), .d(new_n228), .o1(new_n235));
  nano22aa1n02x4               g140(.a(new_n234), .b(new_n231), .c(new_n232), .out0(new_n236));
  aobi12aa1n02x5               g141(.a(new_n236), .b(new_n199), .c(new_n228), .out0(new_n237));
  norb02aa1n02x5               g142(.a(new_n235), .b(new_n237), .out0(\s[21] ));
  xorc02aa1n12x5               g143(.a(\a[22] ), .b(\b[21] ), .out0(new_n239));
  nor042aa1n06x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  norp02aa1n02x5               g145(.a(new_n239), .b(new_n240), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\a[21] ), .o1(new_n242));
  oaib12aa1n02x7               g147(.a(new_n235), .b(\b[20] ), .c(new_n242), .out0(new_n243));
  aoi022aa1n02x5               g148(.a(new_n243), .b(new_n239), .c(new_n235), .d(new_n241), .o1(\s[22] ));
  nand22aa1n12x5               g149(.a(new_n239), .b(new_n234), .o1(new_n245));
  nano32aa1n02x4               g150(.a(new_n245), .b(new_n211), .c(new_n215), .d(new_n221), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n240), .o1(new_n247));
  oao003aa1n03x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .carry(new_n248));
  aoai13aa1n12x5               g153(.a(new_n248), .b(new_n245), .c(new_n231), .d(new_n232), .o1(new_n249));
  tech160nm_fixorc02aa1n03p5x5 g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n249), .c(new_n199), .d(new_n246), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n245), .o1(new_n252));
  nanb02aa1n02x5               g157(.a(new_n250), .b(new_n248), .out0(new_n253));
  aoi012aa1n02x5               g158(.a(new_n253), .b(new_n233), .c(new_n252), .o1(new_n254));
  aobi12aa1n02x5               g159(.a(new_n254), .b(new_n199), .c(new_n246), .out0(new_n255));
  norb02aa1n02x5               g160(.a(new_n251), .b(new_n255), .out0(\s[23] ));
  tech160nm_fixorc02aa1n03p5x5 g161(.a(\a[24] ), .b(\b[23] ), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  norp02aa1n02x5               g163(.a(new_n257), .b(new_n258), .o1(new_n259));
  oai012aa1n02x7               g164(.a(new_n251), .b(\b[22] ), .c(\a[23] ), .o1(new_n260));
  aoi022aa1n02x5               g165(.a(new_n260), .b(new_n257), .c(new_n251), .d(new_n259), .o1(\s[24] ));
  and002aa1n02x7               g166(.a(new_n257), .b(new_n250), .o(new_n262));
  nano22aa1n12x5               g167(.a(new_n227), .b(new_n252), .c(new_n262), .out0(new_n263));
  nand02aa1d04x5               g168(.a(new_n199), .b(new_n263), .o1(new_n264));
  inv000aa1n03x5               g169(.a(new_n258), .o1(new_n265));
  oao003aa1n02x5               g170(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .carry(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  aoi012aa1n02x5               g172(.a(new_n267), .b(new_n249), .c(new_n262), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  aob012aa1n03x5               g174(.a(new_n269), .b(new_n264), .c(new_n268), .out0(new_n270));
  aoi112aa1n02x5               g175(.a(new_n269), .b(new_n267), .c(new_n249), .d(new_n262), .o1(new_n271));
  aobi12aa1n02x7               g176(.a(new_n270), .b(new_n271), .c(new_n264), .out0(\s[25] ));
  xorc02aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .out0(new_n273));
  nor042aa1n03x5               g178(.a(\b[24] ), .b(\a[25] ), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n273), .b(new_n274), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n274), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n269), .o1(new_n277));
  aoai13aa1n02x5               g182(.a(new_n276), .b(new_n277), .c(new_n264), .d(new_n268), .o1(new_n278));
  aoi022aa1n02x5               g183(.a(new_n278), .b(new_n273), .c(new_n270), .d(new_n275), .o1(\s[26] ));
  and002aa1n02x7               g184(.a(new_n273), .b(new_n269), .o(new_n280));
  aoai13aa1n09x5               g185(.a(new_n280), .b(new_n267), .c(new_n249), .d(new_n262), .o1(new_n281));
  nano32aa1n06x5               g186(.a(new_n227), .b(new_n280), .c(new_n252), .d(new_n262), .out0(new_n282));
  nand02aa1d06x5               g187(.a(new_n199), .b(new_n282), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .c(new_n276), .carry(new_n284));
  nand23aa1n09x5               g189(.a(new_n283), .b(new_n281), .c(new_n284), .o1(new_n285));
  xorc02aa1n12x5               g190(.a(\a[27] ), .b(\b[26] ), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n284), .o1(new_n287));
  aoi112aa1n02x5               g192(.a(new_n286), .b(new_n287), .c(new_n199), .d(new_n282), .o1(new_n288));
  aoi022aa1n02x5               g193(.a(new_n285), .b(new_n286), .c(new_n288), .d(new_n281), .o1(\s[27] ));
  nand22aa1n04x5               g194(.a(new_n285), .b(new_n286), .o1(new_n290));
  xorc02aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .out0(new_n291));
  norp02aa1n02x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  norp02aa1n02x5               g197(.a(new_n291), .b(new_n292), .o1(new_n293));
  aoi012aa1n06x5               g198(.a(new_n287), .b(new_n199), .c(new_n282), .o1(new_n294));
  inv000aa1n03x5               g199(.a(new_n292), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n286), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n295), .b(new_n296), .c(new_n294), .d(new_n281), .o1(new_n297));
  aoi022aa1n03x5               g202(.a(new_n297), .b(new_n291), .c(new_n290), .d(new_n293), .o1(\s[28] ));
  and002aa1n02x5               g203(.a(new_n291), .b(new_n286), .o(new_n299));
  nand22aa1n03x5               g204(.a(new_n285), .b(new_n299), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n299), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .c(new_n295), .carry(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n301), .c(new_n294), .d(new_n281), .o1(new_n303));
  xorc02aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .out0(new_n304));
  norb02aa1n02x5               g209(.a(new_n302), .b(new_n304), .out0(new_n305));
  aoi022aa1n03x5               g210(.a(new_n303), .b(new_n304), .c(new_n300), .d(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g212(.a(new_n296), .b(new_n291), .c(new_n304), .out0(new_n308));
  nand22aa1n03x5               g213(.a(new_n285), .b(new_n308), .o1(new_n309));
  inv000aa1n02x5               g214(.a(new_n308), .o1(new_n310));
  inv000aa1d42x5               g215(.a(\b[28] ), .o1(new_n311));
  inv000aa1d42x5               g216(.a(\a[29] ), .o1(new_n312));
  oaib12aa1n02x5               g217(.a(new_n302), .b(\b[28] ), .c(new_n312), .out0(new_n313));
  oaib12aa1n02x5               g218(.a(new_n313), .b(new_n311), .c(\a[29] ), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n310), .c(new_n294), .d(new_n281), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .out0(new_n316));
  oaoi13aa1n02x5               g221(.a(new_n316), .b(new_n313), .c(new_n312), .d(new_n311), .o1(new_n317));
  aoi022aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n309), .d(new_n317), .o1(\s[30] ));
  nano32aa1n02x4               g223(.a(new_n296), .b(new_n316), .c(new_n291), .d(new_n304), .out0(new_n319));
  nand22aa1n04x5               g224(.a(new_n285), .b(new_n319), .o1(new_n320));
  aoi022aa1n02x5               g225(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n321));
  norb02aa1n02x5               g226(.a(\b[30] ), .b(\a[31] ), .out0(new_n322));
  obai22aa1n02x7               g227(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n323));
  aoi112aa1n02x5               g228(.a(new_n323), .b(new_n322), .c(new_n313), .d(new_n321), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  inv000aa1n02x5               g230(.a(new_n319), .o1(new_n326));
  norp02aa1n02x5               g231(.a(\b[29] ), .b(\a[30] ), .o1(new_n327));
  aoi012aa1n02x5               g232(.a(new_n327), .b(new_n313), .c(new_n321), .o1(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n326), .c(new_n294), .d(new_n281), .o1(new_n329));
  aoi022aa1n03x5               g234(.a(new_n329), .b(new_n325), .c(new_n320), .d(new_n324), .o1(\s[31] ));
  xobna2aa1n03x5               g235(.a(new_n105), .b(new_n101), .c(new_n99), .out0(\s[3] ));
  nanp02aa1n02x5               g236(.a(new_n101), .b(new_n99), .o1(new_n332));
  nanb02aa1n02x5               g237(.a(new_n105), .b(new_n332), .out0(new_n333));
  xorc02aa1n02x5               g238(.a(\a[4] ), .b(\b[3] ), .out0(new_n334));
  xnbna2aa1n03x5               g239(.a(new_n334), .b(new_n333), .c(new_n103), .out0(\s[4] ));
  nanp02aa1n02x5               g240(.a(\b[4] ), .b(\a[5] ), .o1(new_n336));
  and002aa1n02x5               g241(.a(\b[3] ), .b(\a[4] ), .o(new_n337));
  nano22aa1n02x4               g242(.a(new_n337), .b(new_n110), .c(new_n336), .out0(new_n338));
  nanp02aa1n02x5               g243(.a(new_n107), .b(new_n338), .o1(new_n339));
  aboi22aa1n03x5               g244(.a(new_n337), .b(new_n107), .c(new_n110), .d(new_n336), .out0(new_n340));
  norb02aa1n02x5               g245(.a(new_n339), .b(new_n340), .out0(\s[5] ));
  norb02aa1n02x5               g246(.a(new_n124), .b(new_n116), .out0(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n342), .b(new_n339), .c(new_n110), .out0(\s[6] ));
  inv000aa1d42x5               g248(.a(new_n116), .o1(new_n344));
  aob012aa1n02x5               g249(.a(new_n342), .b(new_n339), .c(new_n110), .out0(new_n345));
  xorc02aa1n02x5               g250(.a(\a[7] ), .b(\b[6] ), .out0(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n346), .b(new_n345), .c(new_n344), .out0(\s[7] ));
  inv000aa1d42x5               g252(.a(new_n115), .o1(new_n348));
  aob012aa1n02x5               g253(.a(new_n346), .b(new_n345), .c(new_n344), .out0(new_n349));
  norb02aa1n02x5               g254(.a(new_n122), .b(new_n112), .out0(new_n350));
  xnbna2aa1n03x5               g255(.a(new_n350), .b(new_n349), .c(new_n348), .out0(\s[8] ));
  nanp02aa1n02x5               g256(.a(new_n118), .b(new_n107), .o1(new_n352));
  norp03aa1n02x5               g257(.a(new_n125), .b(new_n126), .c(new_n119), .o1(new_n353));
  nano22aa1n02x4               g258(.a(new_n353), .b(new_n123), .c(new_n129), .out0(new_n354));
  aboi22aa1n03x5               g259(.a(new_n190), .b(new_n130), .c(new_n354), .d(new_n352), .out0(\s[9] ));
endmodule


