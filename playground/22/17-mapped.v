// Benchmark "adder" written by ABC on Wed Jul 17 23:22:05 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n339, new_n341, new_n344, new_n345, new_n348, new_n350,
    new_n351;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nor002aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n09x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nona22aa1n02x4               g006(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n102));
  xorc02aa1n02x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  nor002aa1n20x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanb03aa1n02x5               g010(.a(new_n104), .b(new_n105), .c(new_n100), .out0(new_n106));
  nanb03aa1n06x5               g011(.a(new_n106), .b(new_n102), .c(new_n103), .out0(new_n107));
  inv000aa1d42x5               g012(.a(new_n104), .o1(new_n108));
  oao003aa1n02x5               g013(.a(\a[4] ), .b(\b[3] ), .c(new_n108), .carry(new_n109));
  nor002aa1n16x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  tech160nm_finand02aa1n05x5   g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  norb02aa1n02x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  xnrc02aa1n12x5               g017(.a(\b[4] ), .b(\a[5] ), .out0(new_n113));
  xorc02aa1n12x5               g018(.a(\a[8] ), .b(\b[7] ), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .out0(new_n115));
  nona23aa1n02x4               g020(.a(new_n114), .b(new_n112), .c(new_n115), .d(new_n113), .out0(new_n116));
  xorc02aa1n12x5               g021(.a(\a[7] ), .b(\b[6] ), .out0(new_n117));
  inv000aa1d42x5               g022(.a(new_n110), .o1(new_n118));
  nor002aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  aob012aa1n02x5               g024(.a(new_n118), .b(new_n119), .c(new_n111), .out0(new_n120));
  orn002aa1n02x5               g025(.a(\a[7] ), .b(\b[6] ), .o(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  aoi013aa1n02x4               g027(.a(new_n122), .b(new_n120), .c(new_n117), .d(new_n114), .o1(new_n123));
  aoai13aa1n06x5               g028(.a(new_n123), .b(new_n116), .c(new_n107), .d(new_n109), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n97), .b(new_n98), .c(new_n124), .d(new_n125), .o1(new_n126));
  norb03aa1n02x5               g031(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n127));
  xnrc02aa1n02x5               g032(.a(\b[3] ), .b(\a[4] ), .out0(new_n128));
  oai013aa1n03x5               g033(.a(new_n109), .b(new_n127), .c(new_n106), .d(new_n128), .o1(new_n129));
  inv000aa1d42x5               g034(.a(new_n113), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n117), .b(new_n114), .o1(new_n131));
  nano22aa1n03x7               g036(.a(new_n131), .b(new_n130), .c(new_n112), .out0(new_n132));
  inv030aa1n03x5               g037(.a(new_n123), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n125), .b(new_n133), .c(new_n129), .d(new_n132), .o1(new_n134));
  norp02aa1n02x5               g039(.a(new_n97), .b(new_n98), .o1(new_n135));
  nand42aa1n02x5               g040(.a(new_n134), .b(new_n135), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(new_n126), .b(new_n136), .o1(\s[10] ));
  nand42aa1n03x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nor022aa1n12x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  tech160nm_fiaoi012aa1n03p5x5 g044(.a(new_n139), .b(\a[10] ), .c(\b[9] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(new_n140), .b(new_n138), .o1(new_n141));
  nanb02aa1n06x5               g046(.a(new_n139), .b(new_n138), .out0(new_n142));
  aob012aa1n02x5               g047(.a(new_n136), .b(\b[9] ), .c(\a[10] ), .out0(new_n143));
  aboi22aa1n03x5               g048(.a(new_n141), .b(new_n136), .c(new_n143), .d(new_n142), .out0(\s[11] ));
  norp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  and002aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o(new_n146));
  nor042aa1n02x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  aoi113aa1n03x7               g052(.a(new_n147), .b(new_n139), .c(new_n136), .d(new_n140), .e(new_n138), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n139), .o1(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n141), .c(new_n134), .d(new_n135), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n148), .b(new_n147), .c(new_n150), .o1(\s[12] ));
  nona23aa1d18x5               g056(.a(new_n147), .b(new_n125), .c(new_n97), .d(new_n142), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n133), .c(new_n129), .d(new_n132), .o1(new_n154));
  oai012aa1n02x5               g059(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .o1(new_n155));
  oai022aa1n02x5               g060(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n156));
  nona23aa1n09x5               g061(.a(new_n140), .b(new_n156), .c(new_n155), .d(new_n146), .out0(new_n157));
  oab012aa1n06x5               g062(.a(new_n145), .b(new_n149), .c(new_n146), .out0(new_n158));
  nand22aa1n03x5               g063(.a(new_n157), .b(new_n158), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n159), .b(new_n154), .out0(new_n160));
  xorc02aa1n02x5               g065(.a(\a[13] ), .b(\b[12] ), .out0(new_n161));
  nano22aa1n02x4               g066(.a(new_n161), .b(new_n157), .c(new_n158), .out0(new_n162));
  aoi022aa1n02x5               g067(.a(new_n160), .b(new_n161), .c(new_n154), .d(new_n162), .o1(\s[13] ));
  inv000aa1d42x5               g068(.a(\a[13] ), .o1(new_n164));
  nanb02aa1n12x5               g069(.a(\b[12] ), .b(new_n164), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n161), .b(new_n159), .c(new_n124), .d(new_n153), .o1(new_n166));
  xorc02aa1n02x5               g071(.a(\a[14] ), .b(\b[13] ), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n166), .c(new_n165), .out0(\s[14] ));
  inv000aa1d42x5               g073(.a(\a[14] ), .o1(new_n169));
  xroi22aa1d04x5               g074(.a(new_n164), .b(\b[12] ), .c(new_n169), .d(\b[13] ), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n159), .c(new_n124), .d(new_n153), .o1(new_n171));
  oaoi03aa1n12x5               g076(.a(\a[14] ), .b(\b[13] ), .c(new_n165), .o1(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  xorc02aa1n12x5               g078(.a(\a[15] ), .b(\b[14] ), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n171), .c(new_n173), .out0(\s[15] ));
  aoai13aa1n02x5               g080(.a(new_n174), .b(new_n172), .c(new_n160), .d(new_n170), .o1(new_n176));
  xorc02aa1n02x5               g081(.a(\a[16] ), .b(\b[15] ), .out0(new_n177));
  nor042aa1n06x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  norp02aa1n02x5               g083(.a(new_n177), .b(new_n178), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n178), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n174), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n180), .b(new_n181), .c(new_n171), .d(new_n173), .o1(new_n182));
  aoi022aa1n02x5               g087(.a(new_n182), .b(new_n177), .c(new_n176), .d(new_n179), .o1(\s[16] ));
  nanp02aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  xnrc02aa1n02x5               g089(.a(\b[15] ), .b(\a[16] ), .out0(new_n185));
  nano22aa1n03x7               g090(.a(new_n185), .b(new_n180), .c(new_n184), .out0(new_n186));
  nano22aa1d15x5               g091(.a(new_n152), .b(new_n170), .c(new_n186), .out0(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n133), .c(new_n132), .d(new_n129), .o1(new_n188));
  nand23aa1n03x5               g093(.a(new_n172), .b(new_n174), .c(new_n177), .o1(new_n189));
  oao003aa1n02x5               g094(.a(\a[16] ), .b(\b[15] ), .c(new_n180), .carry(new_n190));
  nanp02aa1n02x5               g095(.a(new_n189), .b(new_n190), .o1(new_n191));
  aoi013aa1n06x5               g096(.a(new_n191), .b(new_n159), .c(new_n170), .d(new_n186), .o1(new_n192));
  nanp02aa1n09x5               g097(.a(new_n188), .b(new_n192), .o1(new_n193));
  tech160nm_fixorc02aa1n03p5x5 g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  nanb03aa1n02x5               g099(.a(new_n194), .b(new_n189), .c(new_n190), .out0(new_n195));
  aoi013aa1n02x4               g100(.a(new_n195), .b(new_n159), .c(new_n170), .d(new_n186), .o1(new_n196));
  aoi022aa1n02x5               g101(.a(new_n193), .b(new_n194), .c(new_n188), .d(new_n196), .o1(\s[17] ));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  inv000aa1d42x5               g103(.a(\b[16] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(new_n199), .b(new_n198), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(new_n170), .b(new_n186), .o1(new_n201));
  aobi12aa1n02x7               g106(.a(new_n190), .b(new_n186), .c(new_n172), .out0(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n201), .c(new_n157), .d(new_n158), .o1(new_n203));
  aoai13aa1n02x5               g108(.a(new_n194), .b(new_n203), .c(new_n124), .d(new_n187), .o1(new_n204));
  nor042aa1n02x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nanp02aa1n04x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  norb02aa1n06x4               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n204), .c(new_n200), .out0(\s[18] ));
  and002aa1n02x5               g113(.a(new_n194), .b(new_n207), .o(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n203), .c(new_n124), .d(new_n187), .o1(new_n210));
  aoi013aa1n09x5               g115(.a(new_n205), .b(new_n206), .c(new_n198), .d(new_n199), .o1(new_n211));
  nor042aa1n09x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand02aa1n03x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n210), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n02x5               g121(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n214), .b(new_n217), .c(new_n193), .d(new_n209), .o1(new_n218));
  norp02aa1n04x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand02aa1n03x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  inv000aa1d42x5               g126(.a(\a[19] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[18] ), .o1(new_n223));
  aboi22aa1n03x5               g128(.a(new_n219), .b(new_n220), .c(new_n222), .d(new_n223), .out0(new_n224));
  inv000aa1n03x5               g129(.a(new_n212), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n214), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n225), .b(new_n226), .c(new_n210), .d(new_n211), .o1(new_n227));
  aoi022aa1n02x7               g132(.a(new_n227), .b(new_n221), .c(new_n218), .d(new_n224), .o1(\s[20] ));
  nano23aa1n06x5               g133(.a(new_n212), .b(new_n219), .c(new_n220), .d(new_n213), .out0(new_n229));
  nand23aa1n06x5               g134(.a(new_n229), .b(new_n194), .c(new_n207), .o1(new_n230));
  tech160nm_fiaoi012aa1n05x5   g135(.a(new_n230), .b(new_n188), .c(new_n192), .o1(new_n231));
  nona23aa1n08x5               g136(.a(new_n220), .b(new_n213), .c(new_n212), .d(new_n219), .out0(new_n232));
  oaoi03aa1n09x5               g137(.a(\a[20] ), .b(\b[19] ), .c(new_n225), .o1(new_n233));
  inv040aa1n02x5               g138(.a(new_n233), .o1(new_n234));
  oai012aa1d24x5               g139(.a(new_n234), .b(new_n232), .c(new_n211), .o1(new_n235));
  tech160nm_fixnrc02aa1n04x5   g140(.a(\b[20] ), .b(\a[21] ), .out0(new_n236));
  oabi12aa1n02x5               g141(.a(new_n236), .b(new_n231), .c(new_n235), .out0(new_n237));
  oai112aa1n02x5               g142(.a(new_n234), .b(new_n236), .c(new_n232), .d(new_n211), .o1(new_n238));
  oa0012aa1n02x5               g143(.a(new_n237), .b(new_n231), .c(new_n238), .o(\s[21] ));
  xnrc02aa1n12x5               g144(.a(\b[21] ), .b(\a[22] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  nor042aa1n06x5               g146(.a(\b[20] ), .b(\a[21] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n240), .b(new_n242), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n230), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n203), .c(new_n124), .d(new_n187), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n235), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n242), .o1(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n236), .c(new_n245), .d(new_n246), .o1(new_n248));
  aoi022aa1n02x5               g153(.a(new_n248), .b(new_n241), .c(new_n237), .d(new_n243), .o1(\s[22] ));
  nor042aa1n06x5               g154(.a(new_n240), .b(new_n236), .o1(new_n250));
  norb02aa1n03x5               g155(.a(new_n250), .b(new_n230), .out0(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n203), .c(new_n124), .d(new_n187), .o1(new_n252));
  oaoi03aa1n12x5               g157(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .o1(new_n253));
  tech160nm_fiaoi012aa1n04x5   g158(.a(new_n253), .b(new_n235), .c(new_n250), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  xorc02aa1n12x5               g160(.a(\a[23] ), .b(\b[22] ), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n255), .c(new_n193), .d(new_n251), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(new_n256), .b(new_n253), .c(new_n235), .d(new_n250), .o1(new_n258));
  aobi12aa1n02x7               g163(.a(new_n257), .b(new_n258), .c(new_n252), .out0(\s[23] ));
  xorc02aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  nor042aa1n06x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  norp02aa1n02x5               g166(.a(new_n260), .b(new_n261), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n261), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n256), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n263), .b(new_n264), .c(new_n252), .d(new_n254), .o1(new_n265));
  aoi022aa1n02x5               g170(.a(new_n265), .b(new_n260), .c(new_n257), .d(new_n262), .o1(\s[24] ));
  nano32aa1n03x7               g171(.a(new_n230), .b(new_n260), .c(new_n250), .d(new_n256), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n267), .b(new_n203), .c(new_n124), .d(new_n187), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n250), .b(new_n233), .c(new_n229), .d(new_n217), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n253), .o1(new_n270));
  and002aa1n02x5               g175(.a(new_n260), .b(new_n256), .o(new_n271));
  inv020aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[24] ), .b(\b[23] ), .c(new_n263), .carry(new_n273));
  aoai13aa1n12x5               g178(.a(new_n273), .b(new_n272), .c(new_n269), .d(new_n270), .o1(new_n274));
  xorc02aa1n12x5               g179(.a(\a[25] ), .b(\b[24] ), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n274), .c(new_n193), .d(new_n267), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n271), .b(new_n253), .c(new_n235), .d(new_n250), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n275), .o1(new_n278));
  and003aa1n02x5               g183(.a(new_n277), .b(new_n278), .c(new_n273), .o(new_n279));
  aobi12aa1n03x7               g184(.a(new_n276), .b(new_n279), .c(new_n268), .out0(\s[25] ));
  xorc02aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .out0(new_n281));
  norp02aa1n02x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  norp02aa1n02x5               g187(.a(new_n281), .b(new_n282), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n274), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n282), .o1(new_n285));
  aoai13aa1n02x5               g190(.a(new_n285), .b(new_n278), .c(new_n268), .d(new_n284), .o1(new_n286));
  aoi022aa1n02x5               g191(.a(new_n286), .b(new_n281), .c(new_n276), .d(new_n283), .o1(\s[26] ));
  and002aa1n02x5               g192(.a(new_n281), .b(new_n275), .o(new_n288));
  inv000aa1n02x5               g193(.a(new_n288), .o1(new_n289));
  nano23aa1n06x5               g194(.a(new_n289), .b(new_n230), .c(new_n271), .d(new_n250), .out0(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n203), .c(new_n124), .d(new_n187), .o1(new_n291));
  nanp02aa1n02x5               g196(.a(\b[25] ), .b(\a[26] ), .o1(new_n292));
  oai022aa1n02x5               g197(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(new_n293), .b(new_n292), .o1(new_n294));
  aoai13aa1n04x5               g199(.a(new_n294), .b(new_n289), .c(new_n277), .d(new_n273), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n295), .c(new_n193), .d(new_n290), .o1(new_n297));
  aoi122aa1n02x5               g202(.a(new_n296), .b(new_n292), .c(new_n293), .d(new_n274), .e(new_n288), .o1(new_n298));
  aobi12aa1n02x5               g203(.a(new_n297), .b(new_n298), .c(new_n291), .out0(\s[27] ));
  xorc02aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .out0(new_n300));
  nor042aa1n04x5               g205(.a(\b[26] ), .b(\a[27] ), .o1(new_n301));
  norp02aa1n02x5               g206(.a(new_n300), .b(new_n301), .o1(new_n302));
  aoi022aa1n12x5               g207(.a(new_n274), .b(new_n288), .c(new_n292), .d(new_n293), .o1(new_n303));
  inv000aa1n06x5               g208(.a(new_n301), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n296), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n304), .b(new_n305), .c(new_n303), .d(new_n291), .o1(new_n306));
  aoi022aa1n03x5               g211(.a(new_n306), .b(new_n300), .c(new_n297), .d(new_n302), .o1(\s[28] ));
  and002aa1n02x5               g212(.a(new_n300), .b(new_n296), .o(new_n308));
  aoai13aa1n02x5               g213(.a(new_n308), .b(new_n295), .c(new_n193), .d(new_n290), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  oao003aa1n03x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n304), .carry(new_n311));
  aoai13aa1n06x5               g216(.a(new_n311), .b(new_n310), .c(new_n303), .d(new_n291), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n311), .b(new_n313), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n309), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1d15x5               g221(.a(new_n305), .b(new_n300), .c(new_n313), .out0(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n295), .c(new_n193), .d(new_n290), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n317), .o1(new_n319));
  tech160nm_fioaoi03aa1n02p5x5 g224(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n320), .o1(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n319), .c(new_n303), .d(new_n291), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .out0(new_n323));
  and002aa1n02x5               g228(.a(\b[28] ), .b(\a[29] ), .o(new_n324));
  oabi12aa1n02x5               g229(.a(new_n323), .b(\a[29] ), .c(\b[28] ), .out0(new_n325));
  oab012aa1n02x4               g230(.a(new_n325), .b(new_n311), .c(new_n324), .out0(new_n326));
  aoi022aa1n02x7               g231(.a(new_n322), .b(new_n323), .c(new_n318), .d(new_n326), .o1(\s[30] ));
  nano32aa1d15x5               g232(.a(new_n305), .b(new_n323), .c(new_n300), .d(new_n313), .out0(new_n328));
  aoai13aa1n02x5               g233(.a(new_n328), .b(new_n295), .c(new_n193), .d(new_n290), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[31] ), .b(\b[30] ), .out0(new_n330));
  inv000aa1d42x5               g235(.a(\a[30] ), .o1(new_n331));
  inv000aa1d42x5               g236(.a(\b[29] ), .o1(new_n332));
  oabi12aa1n02x5               g237(.a(new_n330), .b(\a[30] ), .c(\b[29] ), .out0(new_n333));
  oaoi13aa1n02x5               g238(.a(new_n333), .b(new_n320), .c(new_n331), .d(new_n332), .o1(new_n334));
  inv000aa1d42x5               g239(.a(new_n328), .o1(new_n335));
  oaoi03aa1n02x5               g240(.a(new_n331), .b(new_n332), .c(new_n320), .o1(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n335), .c(new_n303), .d(new_n291), .o1(new_n337));
  aoi022aa1n02x7               g242(.a(new_n337), .b(new_n330), .c(new_n329), .d(new_n334), .o1(\s[31] ));
  norb02aa1n02x5               g243(.a(new_n105), .b(new_n104), .out0(new_n339));
  xobna2aa1n03x5               g244(.a(new_n339), .b(new_n102), .c(new_n100), .out0(\s[3] ));
  oai112aa1n02x5               g245(.a(new_n339), .b(new_n100), .c(new_n101), .d(new_n99), .o1(new_n341));
  xnbna2aa1n03x5               g246(.a(new_n103), .b(new_n341), .c(new_n108), .out0(\s[4] ));
  xnbna2aa1n03x5               g247(.a(new_n130), .b(new_n107), .c(new_n109), .out0(\s[5] ));
  aoai13aa1n02x5               g248(.a(new_n112), .b(new_n119), .c(new_n129), .d(new_n130), .o1(new_n344));
  aoi112aa1n02x5               g249(.a(new_n119), .b(new_n112), .c(new_n129), .d(new_n130), .o1(new_n345));
  norb02aa1n02x5               g250(.a(new_n344), .b(new_n345), .out0(\s[6] ));
  xnbna2aa1n03x5               g251(.a(new_n117), .b(new_n344), .c(new_n118), .out0(\s[7] ));
  aob012aa1n02x5               g252(.a(new_n117), .b(new_n344), .c(new_n118), .out0(new_n348));
  xnbna2aa1n03x5               g253(.a(new_n114), .b(new_n348), .c(new_n121), .out0(\s[8] ));
  nanp02aa1n02x5               g254(.a(new_n132), .b(new_n129), .o1(new_n350));
  aoi113aa1n02x5               g255(.a(new_n125), .b(new_n122), .c(new_n120), .d(new_n117), .e(new_n114), .o1(new_n351));
  aoi022aa1n02x5               g256(.a(new_n124), .b(new_n125), .c(new_n350), .d(new_n351), .o1(\s[9] ));
endmodule


