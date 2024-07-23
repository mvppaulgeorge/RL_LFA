// Benchmark "adder" written by ABC on Wed Jul 17 17:51:04 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n315, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n336, new_n337, new_n338,
    new_n340, new_n342, new_n344, new_n345, new_n346, new_n348, new_n349,
    new_n351;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[2] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[1] ), .o1(new_n101));
  aoi022aa1d18x5               g006(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n102));
  aoi012aa1n02x7               g007(.a(new_n102), .b(new_n100), .c(new_n101), .o1(new_n103));
  tech160nm_fixorc02aa1n04x5   g008(.a(\a[3] ), .b(\b[2] ), .out0(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nor002aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nor002aa1n02x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  oaib12aa1n06x5               g012(.a(new_n107), .b(new_n103), .c(new_n104), .out0(new_n108));
  inv040aa1d32x5               g013(.a(\a[7] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[6] ), .o1(new_n110));
  nand42aa1n08x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  nand42aa1n03x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  oai112aa1n03x5               g017(.a(new_n111), .b(new_n112), .c(\b[5] ), .d(\a[6] ), .o1(new_n113));
  aoi022aa1n06x5               g018(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n114));
  nanp02aa1n04x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n115), .b(\b[4] ), .c(\a[5] ), .o1(new_n116));
  tech160nm_fixorc02aa1n03p5x5 g021(.a(\a[8] ), .b(\b[7] ), .out0(new_n117));
  nano23aa1n02x5               g022(.a(new_n113), .b(new_n116), .c(new_n117), .d(new_n114), .out0(new_n118));
  inv000aa1d42x5               g023(.a(\a[8] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[7] ), .o1(new_n120));
  inv040aa1d30x5               g025(.a(\a[5] ), .o1(new_n121));
  inv040aa1d24x5               g026(.a(\b[4] ), .o1(new_n122));
  nor042aa1d18x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nand42aa1n10x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n123), .c(new_n121), .d(new_n122), .o1(new_n125));
  nand02aa1d04x5               g030(.a(new_n125), .b(new_n111), .o1(new_n126));
  aoi022aa1d24x5               g031(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n127));
  aoi022aa1d24x5               g032(.a(new_n126), .b(new_n127), .c(new_n120), .d(new_n119), .o1(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  xorc02aa1n02x5               g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n129), .c(new_n108), .d(new_n118), .o1(new_n131));
  nor002aa1d32x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand42aa1d28x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  norb02aa1n02x7               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n99), .out0(\s[10] ));
  aoai13aa1n03x5               g040(.a(new_n104), .b(new_n102), .c(new_n100), .d(new_n101), .o1(new_n136));
  nona23aa1n09x5               g041(.a(new_n117), .b(new_n114), .c(new_n113), .d(new_n116), .out0(new_n137));
  aoai13aa1n12x5               g042(.a(new_n128), .b(new_n137), .c(new_n136), .d(new_n107), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[8] ), .b(\a[9] ), .o1(new_n139));
  nano32aa1n03x7               g044(.a(new_n132), .b(new_n99), .c(new_n133), .d(new_n139), .out0(new_n140));
  aoai13aa1n12x5               g045(.a(new_n133), .b(new_n132), .c(new_n97), .d(new_n98), .o1(new_n141));
  inv000aa1d42x5               g046(.a(new_n141), .o1(new_n142));
  xnrc02aa1n12x5               g047(.a(\b[10] ), .b(\a[11] ), .out0(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n142), .c(new_n138), .d(new_n140), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(new_n144), .b(new_n142), .c(new_n138), .d(new_n140), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(\s[11] ));
  nor022aa1n04x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand42aa1n03x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanb02aa1n09x5               g054(.a(new_n148), .b(new_n149), .out0(new_n150));
  norp02aa1n02x5               g055(.a(\b[10] ), .b(\a[11] ), .o1(new_n151));
  aoib12aa1n02x5               g056(.a(new_n151), .b(new_n149), .c(new_n148), .out0(new_n152));
  oai012aa1n03x5               g057(.a(new_n145), .b(\b[10] ), .c(\a[11] ), .o1(new_n153));
  aboi22aa1n03x5               g058(.a(new_n150), .b(new_n153), .c(new_n145), .d(new_n152), .out0(\s[12] ));
  nano23aa1n03x7               g059(.a(new_n150), .b(new_n143), .c(new_n130), .d(new_n134), .out0(new_n155));
  tech160nm_fiaoi012aa1n03p5x5 g060(.a(new_n148), .b(new_n151), .c(new_n149), .o1(new_n156));
  oai013aa1d12x5               g061(.a(new_n156), .b(new_n143), .c(new_n141), .d(new_n150), .o1(new_n157));
  nor002aa1d32x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand02aa1d12x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanb02aa1d36x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n157), .c(new_n138), .d(new_n155), .o1(new_n162));
  aoi112aa1n02x5               g067(.a(new_n161), .b(new_n157), .c(new_n138), .d(new_n155), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(\s[13] ));
  inv000aa1d42x5               g069(.a(new_n158), .o1(new_n165));
  norp02aa1n09x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand02aa1n06x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nanb02aa1n03x5               g072(.a(new_n166), .b(new_n167), .out0(new_n168));
  xobna2aa1n03x5               g073(.a(new_n168), .b(new_n162), .c(new_n165), .out0(\s[14] ));
  nona23aa1d18x5               g074(.a(new_n167), .b(new_n159), .c(new_n158), .d(new_n166), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n157), .c(new_n138), .d(new_n155), .o1(new_n172));
  oaoi03aa1n02x5               g077(.a(\a[14] ), .b(\b[13] ), .c(new_n165), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  xorc02aa1n12x5               g079(.a(\a[15] ), .b(\b[14] ), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n172), .c(new_n174), .out0(\s[15] ));
  aob012aa1n03x5               g081(.a(new_n175), .b(new_n172), .c(new_n174), .out0(new_n177));
  xorc02aa1n12x5               g082(.a(\a[16] ), .b(\b[15] ), .out0(new_n178));
  inv000aa1d42x5               g083(.a(\a[15] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[14] ), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n179), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n181), .b(new_n178), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n175), .o1(new_n183));
  aoai13aa1n03x5               g088(.a(new_n181), .b(new_n183), .c(new_n172), .d(new_n174), .o1(new_n184));
  aoi022aa1n02x7               g089(.a(new_n184), .b(new_n178), .c(new_n177), .d(new_n182), .o1(\s[16] ));
  nor042aa1n02x5               g090(.a(new_n143), .b(new_n150), .o1(new_n186));
  nona23aa1d18x5               g091(.a(new_n175), .b(new_n178), .c(new_n168), .d(new_n160), .out0(new_n187));
  nano22aa1n12x5               g092(.a(new_n187), .b(new_n140), .c(new_n186), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n129), .c(new_n108), .d(new_n118), .o1(new_n189));
  norp02aa1n02x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  nano22aa1n06x5               g095(.a(new_n170), .b(new_n175), .c(new_n178), .out0(new_n191));
  oai122aa1n02x7               g096(.a(new_n167), .b(new_n166), .c(new_n158), .d(new_n180), .e(new_n179), .o1(new_n192));
  aoi022aa1n02x5               g097(.a(new_n192), .b(new_n181), .c(\a[16] ), .d(\b[15] ), .o1(new_n193));
  aoi112aa1n03x5               g098(.a(new_n190), .b(new_n193), .c(new_n157), .d(new_n191), .o1(new_n194));
  nand02aa1d06x5               g099(.a(new_n189), .b(new_n194), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv030aa1d32x5               g101(.a(\a[17] ), .o1(new_n197));
  oaib12aa1n06x5               g102(.a(new_n195), .b(new_n197), .c(\b[16] ), .out0(new_n198));
  nor002aa1d32x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  nanp02aa1n03x5               g105(.a(new_n198), .b(new_n200), .o1(new_n201));
  xorc02aa1n02x5               g106(.a(\a[18] ), .b(\b[17] ), .out0(new_n202));
  norp02aa1n02x5               g107(.a(new_n202), .b(new_n199), .o1(new_n203));
  aoi022aa1n02x5               g108(.a(new_n201), .b(new_n202), .c(new_n198), .d(new_n203), .o1(\s[18] ));
  nand02aa1d04x5               g109(.a(new_n157), .b(new_n191), .o1(new_n205));
  nona22aa1n09x5               g110(.a(new_n205), .b(new_n193), .c(new_n190), .out0(new_n206));
  inv040aa1d32x5               g111(.a(\a[18] ), .o1(new_n207));
  xroi22aa1d06x4               g112(.a(new_n197), .b(\b[16] ), .c(new_n207), .d(\b[17] ), .out0(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n206), .c(new_n138), .d(new_n188), .o1(new_n209));
  oai022aa1n09x5               g114(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n210));
  oaib12aa1n09x5               g115(.a(new_n210), .b(new_n207), .c(\b[17] ), .out0(new_n211));
  nor002aa1d32x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand02aa1d16x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n09x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  tech160nm_fioaoi03aa1n03p5x5 g121(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n214), .b(new_n217), .c(new_n195), .d(new_n208), .o1(new_n218));
  nor002aa1d32x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand22aa1n12x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  aoib12aa1n02x5               g126(.a(new_n212), .b(new_n220), .c(new_n219), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n212), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n214), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n223), .b(new_n224), .c(new_n209), .d(new_n211), .o1(new_n225));
  aoi022aa1n03x5               g130(.a(new_n225), .b(new_n221), .c(new_n218), .d(new_n222), .o1(\s[20] ));
  nano23aa1d12x5               g131(.a(new_n212), .b(new_n219), .c(new_n220), .d(new_n213), .out0(new_n227));
  nand02aa1d04x5               g132(.a(new_n208), .b(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n206), .c(new_n138), .d(new_n188), .o1(new_n230));
  nona23aa1d18x5               g135(.a(new_n220), .b(new_n213), .c(new_n212), .d(new_n219), .out0(new_n231));
  tech160nm_fiaoi012aa1n05x5   g136(.a(new_n219), .b(new_n212), .c(new_n220), .o1(new_n232));
  oai012aa1n18x5               g137(.a(new_n232), .b(new_n231), .c(new_n211), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[20] ), .b(\a[21] ), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n230), .c(new_n234), .out0(\s[21] ));
  aoai13aa1n03x5               g142(.a(new_n236), .b(new_n233), .c(new_n195), .d(new_n229), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[21] ), .b(\a[22] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  nor042aa1d18x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n239), .b(new_n241), .out0(new_n242));
  inv040aa1n08x5               g147(.a(new_n241), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n235), .c(new_n230), .d(new_n234), .o1(new_n244));
  aoi022aa1n03x5               g149(.a(new_n244), .b(new_n240), .c(new_n238), .d(new_n242), .o1(\s[22] ));
  nor042aa1n06x5               g150(.a(new_n239), .b(new_n235), .o1(new_n246));
  and003aa1n02x5               g151(.a(new_n208), .b(new_n246), .c(new_n227), .o(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n206), .c(new_n138), .d(new_n188), .o1(new_n248));
  oaoi03aa1n09x5               g153(.a(\a[22] ), .b(\b[21] ), .c(new_n243), .o1(new_n249));
  aoi012aa1d18x5               g154(.a(new_n249), .b(new_n233), .c(new_n246), .o1(new_n250));
  xnrc02aa1n12x5               g155(.a(\b[22] ), .b(\a[23] ), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  xnbna2aa1n03x5               g157(.a(new_n252), .b(new_n248), .c(new_n250), .out0(\s[23] ));
  inv000aa1d42x5               g158(.a(new_n250), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n252), .b(new_n254), .c(new_n195), .d(new_n247), .o1(new_n255));
  xorc02aa1n12x5               g160(.a(\a[24] ), .b(\b[23] ), .out0(new_n256));
  nor042aa1n09x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  norp02aa1n02x5               g162(.a(new_n256), .b(new_n257), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n257), .o1(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n251), .c(new_n248), .d(new_n250), .o1(new_n260));
  aoi022aa1n03x5               g165(.a(new_n260), .b(new_n256), .c(new_n255), .d(new_n258), .o1(\s[24] ));
  nano32aa1d12x5               g166(.a(new_n228), .b(new_n256), .c(new_n246), .d(new_n252), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n206), .c(new_n138), .d(new_n188), .o1(new_n263));
  inv000aa1n02x5               g168(.a(new_n232), .o1(new_n264));
  aoai13aa1n06x5               g169(.a(new_n246), .b(new_n264), .c(new_n227), .d(new_n217), .o1(new_n265));
  inv000aa1n02x5               g170(.a(new_n249), .o1(new_n266));
  norb02aa1n03x5               g171(.a(new_n256), .b(new_n251), .out0(new_n267));
  inv000aa1n02x5               g172(.a(new_n267), .o1(new_n268));
  oao003aa1n02x5               g173(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .carry(new_n269));
  aoai13aa1n12x5               g174(.a(new_n269), .b(new_n268), .c(new_n265), .d(new_n266), .o1(new_n270));
  xorc02aa1n12x5               g175(.a(\a[25] ), .b(\b[24] ), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n270), .c(new_n195), .d(new_n262), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n267), .b(new_n249), .c(new_n233), .d(new_n246), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n271), .o1(new_n274));
  and003aa1n02x5               g179(.a(new_n273), .b(new_n274), .c(new_n269), .o(new_n275));
  aobi12aa1n02x7               g180(.a(new_n272), .b(new_n275), .c(new_n263), .out0(\s[25] ));
  tech160nm_fixorc02aa1n02p5x5 g181(.a(\a[26] ), .b(\b[25] ), .out0(new_n277));
  nor042aa1n06x5               g182(.a(\b[24] ), .b(\a[25] ), .o1(new_n278));
  norp02aa1n02x5               g183(.a(new_n277), .b(new_n278), .o1(new_n279));
  inv000aa1n02x5               g184(.a(new_n270), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n278), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n274), .c(new_n263), .d(new_n280), .o1(new_n282));
  aoi022aa1n02x7               g187(.a(new_n282), .b(new_n277), .c(new_n272), .d(new_n279), .o1(\s[26] ));
  and002aa1n12x5               g188(.a(new_n277), .b(new_n271), .o(new_n284));
  nano32aa1d15x5               g189(.a(new_n228), .b(new_n284), .c(new_n246), .d(new_n267), .out0(new_n285));
  aoai13aa1n12x5               g190(.a(new_n285), .b(new_n206), .c(new_n138), .d(new_n188), .o1(new_n286));
  nor022aa1n16x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  and002aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n287), .o1(new_n289));
  oao003aa1n06x5               g194(.a(\a[26] ), .b(\b[25] ), .c(new_n281), .carry(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  aoi112aa1n02x7               g196(.a(new_n291), .b(new_n289), .c(new_n270), .d(new_n284), .o1(new_n292));
  aoi012aa1n09x5               g197(.a(new_n291), .b(new_n270), .c(new_n284), .o1(new_n293));
  nanp02aa1n03x5               g198(.a(new_n293), .b(new_n286), .o1(new_n294));
  aoi022aa1n02x7               g199(.a(new_n294), .b(new_n289), .c(new_n286), .d(new_n292), .o1(\s[27] ));
  nanp02aa1n02x5               g200(.a(\b[26] ), .b(\a[27] ), .o1(new_n296));
  inv000aa1n02x5               g201(.a(new_n284), .o1(new_n297));
  aoai13aa1n04x5               g202(.a(new_n290), .b(new_n297), .c(new_n273), .d(new_n269), .o1(new_n298));
  aoai13aa1n02x5               g203(.a(new_n296), .b(new_n298), .c(new_n285), .d(new_n195), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n287), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n288), .c(new_n293), .d(new_n286), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n287), .o1(new_n303));
  aoi022aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n299), .d(new_n303), .o1(\s[28] ));
  inv000aa1d42x5               g209(.a(\a[27] ), .o1(new_n305));
  inv000aa1d42x5               g210(.a(\a[28] ), .o1(new_n306));
  xroi22aa1d06x4               g211(.a(new_n305), .b(\b[26] ), .c(new_n306), .d(\b[27] ), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n298), .c(new_n195), .d(new_n285), .o1(new_n308));
  inv000aa1n02x5               g213(.a(new_n307), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[28] ), .b(\b[27] ), .c(new_n300), .carry(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n309), .c(new_n293), .d(new_n286), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[29] ), .b(\b[28] ), .out0(new_n312));
  norb02aa1n02x5               g217(.a(new_n310), .b(new_n312), .out0(new_n313));
  aoi022aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n308), .d(new_n313), .o1(\s[29] ));
  nanp02aa1n02x5               g219(.a(\b[0] ), .b(\a[1] ), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g221(.a(new_n302), .b(new_n312), .c(new_n289), .o(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n298), .c(new_n195), .d(new_n285), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n317), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n310), .carry(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n319), .c(new_n293), .d(new_n286), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  norb02aa1n02x5               g227(.a(new_n320), .b(new_n322), .out0(new_n323));
  aoi022aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n318), .d(new_n323), .o1(\s[30] ));
  nano22aa1n03x7               g229(.a(new_n309), .b(new_n312), .c(new_n322), .out0(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n298), .c(new_n195), .d(new_n285), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[31] ), .b(\b[30] ), .out0(new_n327));
  and002aa1n02x5               g232(.a(\b[29] ), .b(\a[30] ), .o(new_n328));
  oabi12aa1n02x5               g233(.a(new_n327), .b(\a[30] ), .c(\b[29] ), .out0(new_n329));
  oab012aa1n02x4               g234(.a(new_n329), .b(new_n320), .c(new_n328), .out0(new_n330));
  inv000aa1d42x5               g235(.a(new_n325), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n331), .c(new_n293), .d(new_n286), .o1(new_n333));
  aoi022aa1n03x5               g238(.a(new_n333), .b(new_n327), .c(new_n326), .d(new_n330), .o1(\s[31] ));
  xnrb03aa1n02x5               g239(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  obai22aa1n02x7               g240(.a(new_n115), .b(new_n106), .c(\a[3] ), .d(\b[2] ), .out0(new_n336));
  aoib12aa1n02x5               g241(.a(new_n336), .b(new_n104), .c(new_n103), .out0(new_n337));
  nanp02aa1n03x5               g242(.a(new_n108), .b(new_n115), .o1(new_n338));
  oab012aa1n02x4               g243(.a(new_n337), .b(new_n338), .c(new_n106), .out0(\s[4] ));
  xorc02aa1n02x5               g244(.a(\a[5] ), .b(\b[4] ), .out0(new_n340));
  xobna2aa1n03x5               g245(.a(new_n340), .b(new_n108), .c(new_n115), .out0(\s[5] ));
  oaoi03aa1n03x5               g246(.a(\a[5] ), .b(\b[4] ), .c(new_n338), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  xorc02aa1n02x5               g248(.a(\a[7] ), .b(\b[6] ), .out0(new_n344));
  aoai13aa1n06x5               g249(.a(new_n344), .b(new_n123), .c(new_n342), .d(new_n124), .o1(new_n345));
  aoi112aa1n02x5               g250(.a(new_n123), .b(new_n344), .c(new_n342), .d(new_n124), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n345), .b(new_n346), .out0(\s[7] ));
  nanp02aa1n02x5               g252(.a(new_n345), .b(new_n111), .o1(new_n348));
  norb02aa1n02x5               g253(.a(new_n111), .b(new_n117), .out0(new_n349));
  aoi022aa1n02x5               g254(.a(new_n348), .b(new_n117), .c(new_n345), .d(new_n349), .o1(\s[8] ));
  aoi112aa1n02x5               g255(.a(new_n129), .b(new_n130), .c(new_n118), .d(new_n108), .o1(new_n351));
  aoi012aa1n02x5               g256(.a(new_n351), .b(new_n138), .c(new_n130), .o1(\s[9] ));
endmodule

