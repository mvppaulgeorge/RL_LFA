// Benchmark "adder" written by ABC on Wed Jul 17 23:26:22 2024

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
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n321, new_n322, new_n323,
    new_n325, new_n326, new_n327, new_n329, new_n330, new_n331, new_n333,
    new_n334, new_n335, new_n337, new_n339;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1d16x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n06x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  inv000aa1d42x5               g004(.a(new_n99), .o1(new_n100));
  nor022aa1n08x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  nand42aa1n03x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nor022aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nanb03aa1n06x5               g009(.a(new_n103), .b(new_n104), .c(new_n102), .out0(new_n105));
  nand02aa1n03x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  nor042aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  norb03aa1n03x5               g012(.a(new_n104), .b(new_n107), .c(new_n106), .out0(new_n108));
  norp02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nor002aa1n02x5               g014(.a(new_n109), .b(new_n103), .o1(new_n110));
  oai012aa1n04x7               g015(.a(new_n110), .b(new_n108), .c(new_n105), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n02x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nanp03aa1n02x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .o1(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .out0(new_n116));
  nor042aa1n09x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  oab012aa1d24x5               g022(.a(new_n117), .b(\a[6] ), .c(\b[5] ), .out0(new_n118));
  nor042aa1n06x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  tech160nm_fiaoi012aa1n04x5   g024(.a(new_n119), .b(\a[6] ), .c(\b[5] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n118), .b(new_n120), .o1(new_n121));
  nona32aa1n09x5               g026(.a(new_n111), .b(new_n121), .c(new_n116), .d(new_n115), .out0(new_n122));
  tech160nm_fixorc02aa1n02p5x5 g027(.a(\a[8] ), .b(\b[7] ), .out0(new_n123));
  inv000aa1d42x5               g028(.a(new_n118), .o1(new_n124));
  inv000aa1d42x5               g029(.a(new_n119), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[8] ), .b(\b[7] ), .c(new_n125), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[5] ), .b(\a[6] ), .o1(new_n127));
  nano22aa1n02x4               g032(.a(new_n119), .b(new_n113), .c(new_n127), .out0(new_n128));
  aoi013aa1n06x4               g033(.a(new_n126), .b(new_n128), .c(new_n123), .d(new_n124), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(new_n122), .b(new_n129), .o1(new_n130));
  xnrc02aa1n12x5               g035(.a(\b[8] ), .b(\a[9] ), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  aoai13aa1n02x5               g037(.a(new_n100), .b(new_n101), .c(new_n130), .d(new_n132), .o1(new_n133));
  nona22aa1n02x4               g038(.a(new_n98), .b(new_n101), .c(new_n97), .out0(new_n134));
  tech160nm_fiao0012aa1n02p5x5 g039(.a(new_n134), .b(new_n130), .c(new_n132), .o(new_n135));
  nanp02aa1n02x5               g040(.a(new_n135), .b(new_n133), .o1(\s[10] ));
  nor002aa1d32x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  inv020aa1n03x5               g042(.a(new_n137), .o1(new_n138));
  nand02aa1n04x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  aoi022aa1n02x5               g044(.a(new_n135), .b(new_n98), .c(new_n139), .d(new_n138), .o1(new_n140));
  nano22aa1n02x4               g045(.a(new_n137), .b(new_n98), .c(new_n139), .out0(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n134), .c(new_n130), .d(new_n132), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n140), .out0(\s[11] ));
  nor042aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1n04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n142), .c(new_n138), .out0(\s[12] ));
  nona23aa1n06x5               g052(.a(new_n145), .b(new_n139), .c(new_n137), .d(new_n144), .out0(new_n148));
  nanb03aa1n02x5               g053(.a(new_n148), .b(new_n99), .c(new_n132), .out0(new_n149));
  tech160nm_fioai012aa1n03p5x5 g054(.a(new_n145), .b(new_n144), .c(new_n137), .o1(new_n150));
  nanb03aa1n12x5               g055(.a(new_n144), .b(new_n145), .c(new_n139), .out0(new_n151));
  oai112aa1n06x5               g056(.a(new_n138), .b(new_n98), .c(new_n101), .d(new_n97), .o1(new_n152));
  oai012aa1d24x5               g057(.a(new_n150), .b(new_n152), .c(new_n151), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n149), .c(new_n122), .d(new_n129), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  nanp02aa1n04x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanp03aa1n02x5               g064(.a(new_n155), .b(new_n158), .c(new_n159), .o1(new_n160));
  norp02aa1n12x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n09x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n160), .c(new_n158), .out0(\s[14] ));
  nona23aa1d18x5               g069(.a(new_n162), .b(new_n159), .c(new_n157), .d(new_n161), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  oaoi03aa1n02x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .o1(new_n167));
  nor042aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n04x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n02x7               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  aoai13aa1n03x5               g075(.a(new_n170), .b(new_n167), .c(new_n155), .d(new_n166), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n170), .b(new_n167), .c(new_n155), .d(new_n166), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[15] ));
  inv000aa1d42x5               g078(.a(\a[15] ), .o1(new_n174));
  oaib12aa1n02x5               g079(.a(new_n171), .b(\b[14] ), .c(new_n174), .out0(new_n175));
  nor042aa1n03x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanp02aa1n09x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x7               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  norp02aa1n02x5               g083(.a(new_n178), .b(new_n168), .o1(new_n179));
  aoi022aa1n02x5               g084(.a(new_n175), .b(new_n178), .c(new_n171), .d(new_n179), .o1(\s[16] ));
  nano22aa1n06x5               g085(.a(new_n165), .b(new_n170), .c(new_n178), .out0(new_n181));
  nona32aa1n03x5               g086(.a(new_n181), .b(new_n148), .c(new_n100), .d(new_n131), .out0(new_n182));
  nanb02aa1n02x5               g087(.a(new_n182), .b(new_n130), .out0(new_n183));
  nanb03aa1n02x5               g088(.a(new_n176), .b(new_n177), .c(new_n169), .out0(new_n184));
  oai122aa1n02x7               g089(.a(new_n162), .b(new_n161), .c(new_n157), .d(\b[14] ), .e(\a[15] ), .o1(new_n185));
  aoi012aa1n02x5               g090(.a(new_n176), .b(new_n168), .c(new_n177), .o1(new_n186));
  tech160nm_fioai012aa1n04x5   g091(.a(new_n186), .b(new_n185), .c(new_n184), .o1(new_n187));
  aoi012aa1n12x5               g092(.a(new_n187), .b(new_n153), .c(new_n181), .o1(new_n188));
  aoai13aa1n12x5               g093(.a(new_n188), .b(new_n182), .c(new_n122), .d(new_n129), .o1(new_n189));
  xorc02aa1n12x5               g094(.a(\a[17] ), .b(\b[16] ), .out0(new_n190));
  aoi112aa1n02x5               g095(.a(new_n190), .b(new_n187), .c(new_n153), .d(new_n181), .o1(new_n191));
  aoi022aa1n02x5               g096(.a(new_n191), .b(new_n183), .c(new_n189), .d(new_n190), .o1(\s[17] ));
  norp02aa1n02x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[18] ), .b(\b[17] ), .out0(new_n194));
  aoai13aa1n02x5               g099(.a(new_n194), .b(new_n193), .c(new_n189), .d(new_n190), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n193), .b(new_n194), .c(new_n189), .d(new_n190), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(\s[18] ));
  and002aa1n02x5               g102(.a(new_n194), .b(new_n190), .o(new_n198));
  nand22aa1n03x5               g103(.a(new_n189), .b(new_n198), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  oaih22aa1n04x5               g105(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(new_n201), .b(new_n200), .o1(new_n202));
  xorc02aa1n12x5               g107(.a(\a[19] ), .b(\b[18] ), .out0(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n199), .c(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n03x5               g110(.a(new_n203), .b(new_n199), .c(new_n202), .out0(new_n206));
  inv000aa1d42x5               g111(.a(\a[19] ), .o1(new_n207));
  inv000aa1d42x5               g112(.a(\b[18] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(new_n208), .b(new_n207), .o1(new_n209));
  inv040aa1n02x5               g114(.a(new_n203), .o1(new_n210));
  aoai13aa1n02x5               g115(.a(new_n209), .b(new_n210), .c(new_n199), .d(new_n202), .o1(new_n211));
  xorc02aa1n02x5               g116(.a(\a[20] ), .b(\b[19] ), .out0(new_n212));
  inv000aa1d42x5               g117(.a(\a[20] ), .o1(new_n213));
  inv000aa1d42x5               g118(.a(\b[19] ), .o1(new_n214));
  tech160nm_finand02aa1n03p5x5 g119(.a(new_n214), .b(new_n213), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  aoi022aa1n02x5               g121(.a(new_n215), .b(new_n216), .c(new_n208), .d(new_n207), .o1(new_n217));
  aoi022aa1n02x5               g122(.a(new_n211), .b(new_n212), .c(new_n206), .d(new_n217), .o1(\s[20] ));
  nano32aa1n03x7               g123(.a(new_n210), .b(new_n212), .c(new_n190), .d(new_n194), .out0(new_n219));
  oai112aa1n03x5               g124(.a(new_n215), .b(new_n216), .c(new_n208), .d(new_n207), .o1(new_n220));
  nand03aa1n02x5               g125(.a(new_n201), .b(new_n209), .c(new_n200), .o1(new_n221));
  aoi112aa1n02x5               g126(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n215), .b(new_n222), .out0(new_n223));
  oai012aa1n06x5               g128(.a(new_n223), .b(new_n221), .c(new_n220), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n224), .c(new_n189), .d(new_n219), .o1(new_n226));
  aoi112aa1n02x5               g131(.a(new_n225), .b(new_n224), .c(new_n189), .d(new_n219), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n226), .b(new_n227), .out0(\s[21] ));
  inv000aa1d42x5               g133(.a(\a[21] ), .o1(new_n229));
  oaib12aa1n06x5               g134(.a(new_n226), .b(\b[20] ), .c(new_n229), .out0(new_n230));
  xorc02aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .out0(new_n231));
  nor002aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  norp02aa1n02x5               g137(.a(new_n231), .b(new_n232), .o1(new_n233));
  aoi022aa1n03x5               g138(.a(new_n230), .b(new_n231), .c(new_n226), .d(new_n233), .o1(\s[22] ));
  inv000aa1d42x5               g139(.a(\a[22] ), .o1(new_n235));
  xroi22aa1d06x4               g140(.a(new_n229), .b(\b[20] ), .c(new_n235), .d(\b[21] ), .out0(new_n236));
  nand03aa1n02x5               g141(.a(new_n189), .b(new_n219), .c(new_n236), .o1(new_n237));
  aob012aa1n02x5               g142(.a(new_n232), .b(\b[21] ), .c(\a[22] ), .out0(new_n238));
  oa0012aa1n06x5               g143(.a(new_n238), .b(\b[21] ), .c(\a[22] ), .o(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoi012aa1n12x5               g145(.a(new_n240), .b(new_n224), .c(new_n236), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoi013aa1n03x5               g147(.a(new_n242), .b(new_n189), .c(new_n219), .d(new_n236), .o1(new_n243));
  tech160nm_fixorc02aa1n02p5x5 g148(.a(\a[23] ), .b(\b[22] ), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n244), .b(new_n240), .c(new_n224), .d(new_n236), .o1(new_n245));
  aboi22aa1n02x7               g150(.a(new_n243), .b(new_n244), .c(new_n245), .d(new_n237), .out0(\s[23] ));
  aob012aa1n03x5               g151(.a(new_n244), .b(new_n237), .c(new_n241), .out0(new_n247));
  tech160nm_fioaoi03aa1n03p5x5 g152(.a(\a[23] ), .b(\b[22] ), .c(new_n243), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  norp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  aoi022aa1n03x5               g156(.a(new_n248), .b(new_n249), .c(new_n247), .d(new_n251), .o1(\s[24] ));
  inv000aa1n02x5               g157(.a(new_n219), .o1(new_n253));
  and002aa1n02x5               g158(.a(new_n249), .b(new_n244), .o(new_n254));
  nano22aa1n02x4               g159(.a(new_n253), .b(new_n236), .c(new_n254), .out0(new_n255));
  nand02aa1d04x5               g160(.a(new_n189), .b(new_n255), .o1(new_n256));
  inv000aa1d42x5               g161(.a(\a[24] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(\b[23] ), .o1(new_n258));
  oaoi03aa1n02x5               g163(.a(new_n257), .b(new_n258), .c(new_n250), .o1(new_n259));
  aobi12aa1n02x5               g164(.a(new_n259), .b(new_n242), .c(new_n254), .out0(new_n260));
  xorc02aa1n12x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  aob012aa1n03x5               g166(.a(new_n261), .b(new_n256), .c(new_n260), .out0(new_n262));
  aoai13aa1n02x5               g167(.a(new_n254), .b(new_n240), .c(new_n224), .d(new_n236), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n261), .o1(new_n264));
  and003aa1n02x5               g169(.a(new_n263), .b(new_n264), .c(new_n259), .o(new_n265));
  aobi12aa1n02x7               g170(.a(new_n262), .b(new_n265), .c(new_n256), .out0(\s[25] ));
  nor042aa1n03x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n264), .c(new_n256), .d(new_n260), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  norp02aa1n02x5               g175(.a(new_n270), .b(new_n267), .o1(new_n271));
  aoi022aa1n03x5               g176(.a(new_n269), .b(new_n270), .c(new_n262), .d(new_n271), .o1(\s[26] ));
  and002aa1n02x5               g177(.a(new_n270), .b(new_n261), .o(new_n273));
  aob012aa1n09x5               g178(.a(new_n273), .b(new_n263), .c(new_n259), .out0(new_n274));
  nano32aa1n03x7               g179(.a(new_n253), .b(new_n273), .c(new_n236), .d(new_n254), .out0(new_n275));
  nand02aa1n06x5               g180(.a(new_n189), .b(new_n275), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n277));
  nanp03aa1n09x5               g182(.a(new_n274), .b(new_n276), .c(new_n277), .o1(new_n278));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  and002aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n279), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n277), .o1(new_n282));
  aoi112aa1n02x5               g187(.a(new_n281), .b(new_n282), .c(new_n189), .d(new_n275), .o1(new_n283));
  aoi022aa1n02x5               g188(.a(new_n278), .b(new_n281), .c(new_n274), .d(new_n283), .o1(\s[27] ));
  nanp02aa1n03x5               g189(.a(new_n278), .b(new_n281), .o1(new_n285));
  tech160nm_fiaoi012aa1n05x5   g190(.a(new_n282), .b(new_n189), .c(new_n275), .o1(new_n286));
  inv000aa1n03x5               g191(.a(new_n279), .o1(new_n287));
  aoai13aa1n02x7               g192(.a(new_n287), .b(new_n280), .c(new_n286), .d(new_n274), .o1(new_n288));
  xorc02aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .out0(new_n289));
  norp02aa1n02x5               g194(.a(new_n289), .b(new_n279), .o1(new_n290));
  aoi022aa1n03x5               g195(.a(new_n288), .b(new_n289), .c(new_n285), .d(new_n290), .o1(\s[28] ));
  inv000aa1d42x5               g196(.a(\a[27] ), .o1(new_n292));
  inv000aa1d42x5               g197(.a(\a[28] ), .o1(new_n293));
  xroi22aa1d04x5               g198(.a(new_n292), .b(\b[26] ), .c(new_n293), .d(\b[27] ), .out0(new_n294));
  nanp02aa1n03x5               g199(.a(new_n278), .b(new_n294), .o1(new_n295));
  inv000aa1n03x5               g200(.a(new_n294), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .carry(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n296), .c(new_n286), .d(new_n274), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .out0(new_n299));
  norb02aa1n02x5               g204(.a(new_n297), .b(new_n299), .out0(new_n300));
  aoi022aa1n03x5               g205(.a(new_n298), .b(new_n299), .c(new_n295), .d(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g207(.a(new_n289), .b(new_n299), .c(new_n281), .o(new_n303));
  nand22aa1n03x5               g208(.a(new_n278), .b(new_n303), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n303), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .carry(new_n306));
  aoai13aa1n02x7               g211(.a(new_n306), .b(new_n305), .c(new_n286), .d(new_n274), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .out0(new_n308));
  norb02aa1n02x5               g213(.a(new_n306), .b(new_n308), .out0(new_n309));
  aoi022aa1n03x5               g214(.a(new_n307), .b(new_n308), .c(new_n304), .d(new_n309), .o1(\s[30] ));
  nano22aa1n02x4               g215(.a(new_n296), .b(new_n299), .c(new_n308), .out0(new_n311));
  nanp02aa1n03x5               g216(.a(new_n278), .b(new_n311), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[31] ), .b(\b[30] ), .out0(new_n313));
  and002aa1n02x5               g218(.a(\b[29] ), .b(\a[30] ), .o(new_n314));
  oabi12aa1n02x5               g219(.a(new_n313), .b(\a[30] ), .c(\b[29] ), .out0(new_n315));
  oab012aa1n02x4               g220(.a(new_n315), .b(new_n306), .c(new_n314), .out0(new_n316));
  inv000aa1n02x5               g221(.a(new_n311), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .carry(new_n318));
  aoai13aa1n02x7               g223(.a(new_n318), .b(new_n317), .c(new_n286), .d(new_n274), .o1(new_n319));
  aoi022aa1n03x5               g224(.a(new_n319), .b(new_n313), .c(new_n312), .d(new_n316), .o1(\s[31] ));
  norp02aa1n02x5               g225(.a(new_n108), .b(new_n105), .o1(new_n321));
  oai012aa1n02x5               g226(.a(new_n104), .b(new_n107), .c(new_n106), .o1(new_n322));
  oaib12aa1n02x5               g227(.a(new_n322), .b(new_n103), .c(new_n102), .out0(new_n323));
  norb02aa1n02x5               g228(.a(new_n323), .b(new_n321), .out0(\s[3] ));
  obai22aa1n02x7               g229(.a(new_n114), .b(new_n109), .c(\a[3] ), .d(\b[2] ), .out0(new_n325));
  and002aa1n02x5               g230(.a(new_n111), .b(new_n114), .o(new_n326));
  inv000aa1d42x5               g231(.a(new_n326), .o1(new_n327));
  oa0022aa1n02x5               g232(.a(new_n327), .b(new_n109), .c(new_n321), .d(new_n325), .o(\s[4] ));
  nanb03aa1n02x5               g233(.a(new_n117), .b(new_n112), .c(new_n114), .out0(new_n329));
  oaoi13aa1n02x5               g234(.a(new_n329), .b(new_n110), .c(new_n108), .d(new_n105), .o1(new_n330));
  aboi22aa1n03x5               g235(.a(new_n117), .b(new_n112), .c(new_n111), .d(new_n114), .out0(new_n331));
  norp02aa1n02x5               g236(.a(new_n331), .b(new_n330), .o1(\s[5] ));
  xorc02aa1n02x5               g237(.a(\a[6] ), .b(\b[5] ), .out0(new_n333));
  oai012aa1n02x5               g238(.a(new_n333), .b(new_n330), .c(new_n117), .o1(new_n334));
  norp03aa1n02x5               g239(.a(new_n330), .b(new_n333), .c(new_n117), .o1(new_n335));
  norb02aa1n02x5               g240(.a(new_n334), .b(new_n335), .out0(\s[6] ));
  oai012aa1n02x5               g241(.a(new_n334), .b(\b[5] ), .c(\a[6] ), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanp03aa1n02x5               g243(.a(new_n337), .b(new_n113), .c(new_n125), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n123), .b(new_n339), .c(new_n125), .out0(\s[8] ));
  xnbna2aa1n03x5               g245(.a(new_n132), .b(new_n122), .c(new_n129), .out0(\s[9] ));
endmodule

