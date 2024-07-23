// Benchmark "adder" written by ABC on Thu Jul 18 00:01:24 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n334, new_n335, new_n336, new_n337,
    new_n339, new_n340, new_n343, new_n344, new_n345, new_n346, new_n349,
    new_n351, new_n352;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n03x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand42aa1n08x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d06x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nona22aa1n02x4               g005(.a(new_n99), .b(new_n98), .c(new_n100), .out0(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(\b[3] ), .c(\a[4] ), .o1(new_n102));
  nand22aa1n09x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(\a[4] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[3] ), .o1(new_n105));
  oai022aa1n02x5               g010(.a(new_n104), .b(new_n105), .c(\a[3] ), .d(\b[2] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n101), .b(new_n103), .c(new_n106), .d(new_n102), .out0(new_n107));
  orn002aa1n12x5               g012(.a(\a[3] ), .b(\b[2] ), .o(new_n108));
  oao003aa1n02x5               g013(.a(\a[4] ), .b(\b[3] ), .c(new_n108), .carry(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  tech160nm_fixorc02aa1n03p5x5 g015(.a(\a[5] ), .b(\b[4] ), .out0(new_n111));
  xorc02aa1n12x5               g016(.a(\a[8] ), .b(\b[7] ), .out0(new_n112));
  nor042aa1n12x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  tech160nm_finand02aa1n05x5   g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  nona23aa1n03x5               g020(.a(new_n111), .b(new_n112), .c(new_n110), .d(new_n115), .out0(new_n116));
  tech160nm_finand02aa1n05x5   g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nano22aa1n03x7               g022(.a(new_n113), .b(new_n117), .c(new_n114), .out0(new_n118));
  oai022aa1n06x5               g023(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  inv030aa1n02x5               g024(.a(new_n113), .o1(new_n120));
  oaoi03aa1n09x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .o1(new_n121));
  aoi013aa1n09x5               g026(.a(new_n121), .b(new_n118), .c(new_n112), .d(new_n119), .o1(new_n122));
  aoai13aa1n12x5               g027(.a(new_n122), .b(new_n116), .c(new_n107), .d(new_n109), .o1(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  xnrc02aa1n12x5               g030(.a(\b[9] ), .b(\a[10] ), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  aoai13aa1n09x5               g032(.a(new_n127), .b(new_n97), .c(new_n123), .d(new_n125), .o1(new_n128));
  aoi112aa1n02x5               g033(.a(new_n127), .b(new_n97), .c(new_n123), .d(new_n125), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n128), .b(new_n129), .out0(\s[10] ));
  inv000aa1n06x5               g035(.a(new_n97), .o1(new_n131));
  oaoi03aa1n12x5               g036(.a(\a[10] ), .b(\b[9] ), .c(new_n131), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  tech160nm_fixnrc02aa1n05x5   g038(.a(\b[10] ), .b(\a[11] ), .out0(new_n134));
  inv030aa1n03x5               g039(.a(new_n134), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n128), .c(new_n133), .out0(\s[11] ));
  aob012aa1n03x5               g041(.a(new_n135), .b(new_n128), .c(new_n133), .out0(new_n137));
  xnrc02aa1n03x5               g042(.a(\b[11] ), .b(\a[12] ), .out0(new_n138));
  inv030aa1n03x5               g043(.a(new_n138), .o1(new_n139));
  nor042aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n138), .b(new_n140), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n140), .o1(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n134), .c(new_n128), .d(new_n133), .o1(new_n143));
  aoi022aa1n02x5               g048(.a(new_n143), .b(new_n139), .c(new_n137), .d(new_n141), .o1(\s[12] ));
  nanp02aa1n02x5               g049(.a(new_n107), .b(new_n109), .o1(new_n145));
  xorc02aa1n02x5               g050(.a(\a[6] ), .b(\b[5] ), .out0(new_n146));
  nano32aa1n02x4               g051(.a(new_n115), .b(new_n112), .c(new_n111), .d(new_n146), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n122), .o1(new_n148));
  nona23aa1d24x5               g053(.a(new_n135), .b(new_n139), .c(new_n126), .d(new_n124), .out0(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n148), .c(new_n147), .d(new_n145), .o1(new_n151));
  nona22aa1n03x5               g056(.a(new_n132), .b(new_n134), .c(new_n138), .out0(new_n152));
  oao003aa1n02x5               g057(.a(\a[12] ), .b(\b[11] ), .c(new_n142), .carry(new_n153));
  nand02aa1n02x5               g058(.a(new_n152), .b(new_n153), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n154), .b(new_n151), .out0(new_n155));
  xorc02aa1n02x5               g060(.a(\a[13] ), .b(\b[12] ), .out0(new_n156));
  nano22aa1n02x4               g061(.a(new_n156), .b(new_n152), .c(new_n153), .out0(new_n157));
  aoi022aa1n02x5               g062(.a(new_n155), .b(new_n156), .c(new_n151), .d(new_n157), .o1(\s[13] ));
  inv040aa1d30x5               g063(.a(\a[13] ), .o1(new_n159));
  nanb02aa1n12x5               g064(.a(\b[12] ), .b(new_n159), .out0(new_n160));
  aoai13aa1n02x5               g065(.a(new_n156), .b(new_n154), .c(new_n123), .d(new_n150), .o1(new_n161));
  xorc02aa1n02x5               g066(.a(\a[14] ), .b(\b[13] ), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n161), .c(new_n160), .out0(\s[14] ));
  inv040aa1d32x5               g068(.a(\a[14] ), .o1(new_n164));
  xroi22aa1d06x4               g069(.a(new_n159), .b(\b[12] ), .c(new_n164), .d(\b[13] ), .out0(new_n165));
  aoai13aa1n03x5               g070(.a(new_n165), .b(new_n154), .c(new_n123), .d(new_n150), .o1(new_n166));
  oaoi03aa1n09x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  xorc02aa1n12x5               g073(.a(\a[15] ), .b(\b[14] ), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n166), .c(new_n168), .out0(\s[15] ));
  aoai13aa1n02x5               g075(.a(new_n169), .b(new_n167), .c(new_n155), .d(new_n165), .o1(new_n171));
  xorc02aa1n02x5               g076(.a(\a[16] ), .b(\b[15] ), .out0(new_n172));
  nor042aa1n06x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norp02aa1n02x5               g078(.a(new_n172), .b(new_n173), .o1(new_n174));
  inv000aa1n03x5               g079(.a(new_n173), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n169), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n175), .b(new_n176), .c(new_n166), .d(new_n168), .o1(new_n177));
  aoi022aa1n02x5               g082(.a(new_n177), .b(new_n172), .c(new_n171), .d(new_n174), .o1(\s[16] ));
  nanp02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  tech160nm_fixnrc02aa1n02p5x5 g084(.a(\b[15] ), .b(\a[16] ), .out0(new_n180));
  nano22aa1n03x7               g085(.a(new_n180), .b(new_n175), .c(new_n179), .out0(new_n181));
  nano22aa1d15x5               g086(.a(new_n149), .b(new_n165), .c(new_n181), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n148), .c(new_n147), .d(new_n145), .o1(new_n183));
  nand22aa1n03x5               g088(.a(new_n165), .b(new_n181), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  nanp03aa1n02x5               g090(.a(new_n167), .b(new_n169), .c(new_n172), .o1(new_n186));
  oao003aa1n02x5               g091(.a(\a[16] ), .b(\b[15] ), .c(new_n175), .carry(new_n187));
  nanp02aa1n02x5               g092(.a(new_n186), .b(new_n187), .o1(new_n188));
  aoi012aa1n06x5               g093(.a(new_n188), .b(new_n185), .c(new_n154), .o1(new_n189));
  nanp02aa1n09x5               g094(.a(new_n183), .b(new_n189), .o1(new_n190));
  tech160nm_fixorc02aa1n05x5   g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  nanb03aa1n02x5               g096(.a(new_n191), .b(new_n186), .c(new_n187), .out0(new_n192));
  aoi012aa1n02x5               g097(.a(new_n192), .b(new_n185), .c(new_n154), .o1(new_n193));
  aoi022aa1n02x5               g098(.a(new_n190), .b(new_n191), .c(new_n183), .d(new_n193), .o1(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[16] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  aobi12aa1n02x5               g102(.a(new_n187), .b(new_n181), .c(new_n167), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n184), .c(new_n152), .d(new_n153), .o1(new_n199));
  aoai13aa1n03x5               g104(.a(new_n191), .b(new_n199), .c(new_n123), .d(new_n182), .o1(new_n200));
  nor042aa1n03x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand02aa1n04x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  norb02aa1n06x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n200), .c(new_n197), .out0(\s[18] ));
  and002aa1n02x5               g109(.a(new_n191), .b(new_n203), .o(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n199), .c(new_n123), .d(new_n182), .o1(new_n206));
  aoi013aa1n09x5               g111(.a(new_n201), .b(new_n202), .c(new_n195), .d(new_n196), .o1(new_n207));
  nor042aa1n06x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nand02aa1n03x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n206), .c(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n02x5               g117(.a(\a[18] ), .b(\b[17] ), .c(new_n197), .o1(new_n213));
  aoai13aa1n03x5               g118(.a(new_n210), .b(new_n213), .c(new_n190), .d(new_n205), .o1(new_n214));
  norp02aa1n06x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nand02aa1n04x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  inv000aa1d42x5               g122(.a(\a[19] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\b[18] ), .o1(new_n219));
  aboi22aa1n03x5               g124(.a(new_n215), .b(new_n216), .c(new_n218), .d(new_n219), .out0(new_n220));
  inv000aa1n06x5               g125(.a(new_n208), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n210), .o1(new_n222));
  aoai13aa1n02x5               g127(.a(new_n221), .b(new_n222), .c(new_n206), .d(new_n207), .o1(new_n223));
  aoi022aa1n03x5               g128(.a(new_n223), .b(new_n217), .c(new_n214), .d(new_n220), .o1(\s[20] ));
  nano23aa1n06x5               g129(.a(new_n208), .b(new_n215), .c(new_n216), .d(new_n209), .out0(new_n225));
  nand23aa1n06x5               g130(.a(new_n225), .b(new_n191), .c(new_n203), .o1(new_n226));
  aoi012aa1n06x5               g131(.a(new_n226), .b(new_n183), .c(new_n189), .o1(new_n227));
  nona23aa1n08x5               g132(.a(new_n216), .b(new_n209), .c(new_n208), .d(new_n215), .out0(new_n228));
  oaoi03aa1n09x5               g133(.a(\a[20] ), .b(\b[19] ), .c(new_n221), .o1(new_n229));
  inv040aa1n03x5               g134(.a(new_n229), .o1(new_n230));
  oai012aa1d24x5               g135(.a(new_n230), .b(new_n228), .c(new_n207), .o1(new_n231));
  tech160nm_fixnrc02aa1n04x5   g136(.a(\b[20] ), .b(\a[21] ), .out0(new_n232));
  oabi12aa1n06x5               g137(.a(new_n232), .b(new_n227), .c(new_n231), .out0(new_n233));
  oai112aa1n02x5               g138(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n207), .o1(new_n234));
  oa0012aa1n02x5               g139(.a(new_n233), .b(new_n227), .c(new_n234), .o(\s[21] ));
  tech160nm_fixnrc02aa1n02p5x5 g140(.a(\b[21] ), .b(\a[22] ), .out0(new_n236));
  norp02aa1n02x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n236), .b(new_n237), .out0(new_n238));
  inv000aa1d42x5               g143(.a(new_n226), .o1(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n199), .c(new_n123), .d(new_n182), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n231), .o1(new_n241));
  inv000aa1n03x5               g146(.a(new_n237), .o1(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n232), .c(new_n240), .d(new_n241), .o1(new_n243));
  aboi22aa1n03x5               g148(.a(new_n236), .b(new_n243), .c(new_n233), .d(new_n238), .out0(\s[22] ));
  nor002aa1n04x5               g149(.a(new_n236), .b(new_n232), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n226), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n199), .c(new_n123), .d(new_n182), .o1(new_n247));
  oaoi03aa1n12x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n242), .o1(new_n248));
  tech160nm_fiaoi012aa1n04x5   g153(.a(new_n248), .b(new_n231), .c(new_n245), .o1(new_n249));
  inv000aa1n02x5               g154(.a(new_n249), .o1(new_n250));
  xorc02aa1n12x5               g155(.a(\a[23] ), .b(\b[22] ), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n250), .c(new_n190), .d(new_n246), .o1(new_n252));
  aoi112aa1n02x5               g157(.a(new_n251), .b(new_n248), .c(new_n231), .d(new_n245), .o1(new_n253));
  aobi12aa1n03x7               g158(.a(new_n252), .b(new_n253), .c(new_n247), .out0(\s[23] ));
  xorc02aa1n02x5               g159(.a(\a[24] ), .b(\b[23] ), .out0(new_n255));
  nor042aa1n06x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  norp02aa1n02x5               g161(.a(new_n255), .b(new_n256), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n256), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n251), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n258), .b(new_n259), .c(new_n247), .d(new_n249), .o1(new_n260));
  aoi022aa1n02x5               g165(.a(new_n260), .b(new_n255), .c(new_n252), .d(new_n257), .o1(\s[24] ));
  nano32aa1n02x5               g166(.a(new_n226), .b(new_n255), .c(new_n245), .d(new_n251), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n199), .c(new_n123), .d(new_n182), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n245), .b(new_n229), .c(new_n225), .d(new_n213), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n248), .o1(new_n265));
  and002aa1n02x5               g170(.a(new_n255), .b(new_n251), .o(new_n266));
  inv000aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  oao003aa1n02x5               g172(.a(\a[24] ), .b(\b[23] ), .c(new_n258), .carry(new_n268));
  aoai13aa1n12x5               g173(.a(new_n268), .b(new_n267), .c(new_n264), .d(new_n265), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n269), .c(new_n190), .d(new_n262), .o1(new_n271));
  aoai13aa1n03x5               g176(.a(new_n266), .b(new_n248), .c(new_n231), .d(new_n245), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n270), .o1(new_n273));
  and003aa1n02x5               g178(.a(new_n272), .b(new_n273), .c(new_n268), .o(new_n274));
  aobi12aa1n03x7               g179(.a(new_n271), .b(new_n274), .c(new_n263), .out0(\s[25] ));
  xorc02aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .out0(new_n276));
  norp02aa1n02x5               g181(.a(\b[24] ), .b(\a[25] ), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n276), .b(new_n277), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n269), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n277), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n273), .c(new_n263), .d(new_n279), .o1(new_n281));
  aoi022aa1n03x5               g186(.a(new_n281), .b(new_n276), .c(new_n271), .d(new_n278), .o1(\s[26] ));
  and002aa1n02x5               g187(.a(new_n276), .b(new_n270), .o(new_n283));
  inv000aa1n02x5               g188(.a(new_n283), .o1(new_n284));
  nano23aa1n06x5               g189(.a(new_n284), .b(new_n226), .c(new_n266), .d(new_n245), .out0(new_n285));
  aoai13aa1n09x5               g190(.a(new_n285), .b(new_n199), .c(new_n123), .d(new_n182), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(\b[25] ), .b(\a[26] ), .o1(new_n287));
  oai022aa1n02x5               g192(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n288), .b(new_n287), .o1(new_n289));
  aoai13aa1n04x5               g194(.a(new_n289), .b(new_n284), .c(new_n272), .d(new_n268), .o1(new_n290));
  xorc02aa1n12x5               g195(.a(\a[27] ), .b(\b[26] ), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n290), .c(new_n190), .d(new_n285), .o1(new_n292));
  aoi122aa1n02x5               g197(.a(new_n291), .b(new_n287), .c(new_n288), .d(new_n269), .e(new_n283), .o1(new_n293));
  aobi12aa1n03x7               g198(.a(new_n292), .b(new_n293), .c(new_n286), .out0(\s[27] ));
  tech160nm_fixorc02aa1n03p5x5 g199(.a(\a[28] ), .b(\b[27] ), .out0(new_n295));
  nor042aa1d18x5               g200(.a(\b[26] ), .b(\a[27] ), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n295), .b(new_n296), .o1(new_n297));
  aoi022aa1n06x5               g202(.a(new_n269), .b(new_n283), .c(new_n287), .d(new_n288), .o1(new_n298));
  inv000aa1n06x5               g203(.a(new_n296), .o1(new_n299));
  inv000aa1n02x5               g204(.a(new_n291), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n299), .b(new_n300), .c(new_n298), .d(new_n286), .o1(new_n301));
  aoi022aa1n03x5               g206(.a(new_n301), .b(new_n295), .c(new_n292), .d(new_n297), .o1(\s[28] ));
  and002aa1n02x5               g207(.a(new_n295), .b(new_n291), .o(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n290), .c(new_n190), .d(new_n285), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n303), .o1(new_n305));
  oao003aa1n03x5               g210(.a(\a[28] ), .b(\b[27] ), .c(new_n299), .carry(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n305), .c(new_n298), .d(new_n286), .o1(new_n307));
  tech160nm_fixorc02aa1n03p5x5 g212(.a(\a[29] ), .b(\b[28] ), .out0(new_n308));
  norb02aa1n02x5               g213(.a(new_n306), .b(new_n308), .out0(new_n309));
  aoi022aa1n03x5               g214(.a(new_n307), .b(new_n308), .c(new_n304), .d(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g215(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g216(.a(new_n300), .b(new_n295), .c(new_n308), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n290), .c(new_n190), .d(new_n285), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n312), .o1(new_n314));
  tech160nm_fioaoi03aa1n02p5x5 g219(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n315), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n314), .c(new_n298), .d(new_n286), .o1(new_n317));
  tech160nm_fixorc02aa1n02p5x5 g222(.a(\a[30] ), .b(\b[29] ), .out0(new_n318));
  and002aa1n02x5               g223(.a(\b[28] ), .b(\a[29] ), .o(new_n319));
  oabi12aa1n02x5               g224(.a(new_n318), .b(\a[29] ), .c(\b[28] ), .out0(new_n320));
  oab012aa1n02x4               g225(.a(new_n320), .b(new_n306), .c(new_n319), .out0(new_n321));
  aoi022aa1n02x7               g226(.a(new_n317), .b(new_n318), .c(new_n313), .d(new_n321), .o1(\s[30] ));
  nano32aa1n02x5               g227(.a(new_n300), .b(new_n318), .c(new_n295), .d(new_n308), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n290), .c(new_n190), .d(new_n285), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  inv000aa1d42x5               g230(.a(\a[30] ), .o1(new_n326));
  inv000aa1d42x5               g231(.a(\b[29] ), .o1(new_n327));
  oabi12aa1n02x5               g232(.a(new_n325), .b(\a[30] ), .c(\b[29] ), .out0(new_n328));
  oaoi13aa1n02x5               g233(.a(new_n328), .b(new_n315), .c(new_n326), .d(new_n327), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n323), .o1(new_n330));
  oaoi03aa1n02x5               g235(.a(new_n326), .b(new_n327), .c(new_n315), .o1(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n330), .c(new_n298), .d(new_n286), .o1(new_n332));
  aoi022aa1n03x5               g237(.a(new_n332), .b(new_n325), .c(new_n324), .d(new_n329), .o1(\s[31] ));
  norb03aa1n03x5               g238(.a(new_n99), .b(new_n98), .c(new_n100), .out0(new_n334));
  inv000aa1d42x5               g239(.a(new_n103), .o1(new_n335));
  nano23aa1n02x4               g240(.a(new_n334), .b(new_n335), .c(new_n108), .d(new_n99), .out0(new_n336));
  aoi022aa1n02x5               g241(.a(new_n101), .b(new_n99), .c(new_n108), .d(new_n103), .o1(new_n337));
  norp02aa1n02x5               g242(.a(new_n336), .b(new_n337), .o1(\s[3] ));
  inv000aa1n03x5               g243(.a(new_n336), .o1(new_n339));
  xorc02aa1n02x5               g244(.a(\a[4] ), .b(\b[3] ), .out0(new_n340));
  xnbna2aa1n03x5               g245(.a(new_n340), .b(new_n339), .c(new_n108), .out0(\s[4] ));
  xnbna2aa1n03x5               g246(.a(new_n111), .b(new_n107), .c(new_n109), .out0(\s[5] ));
  norp02aa1n02x5               g247(.a(\b[4] ), .b(\a[5] ), .o1(new_n343));
  aoi012aa1n02x5               g248(.a(new_n343), .b(new_n145), .c(new_n111), .o1(new_n344));
  norp02aa1n02x5               g249(.a(new_n110), .b(new_n343), .o1(new_n345));
  aob012aa1n02x5               g250(.a(new_n345), .b(new_n145), .c(new_n111), .out0(new_n346));
  oai012aa1n02x5               g251(.a(new_n346), .b(new_n344), .c(new_n146), .o1(\s[6] ));
  xnbna2aa1n03x5               g252(.a(new_n115), .b(new_n346), .c(new_n117), .out0(\s[7] ));
  nanp02aa1n02x5               g253(.a(new_n346), .b(new_n118), .o1(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n112), .b(new_n349), .c(new_n120), .out0(\s[8] ));
  nanp02aa1n02x5               g255(.a(new_n147), .b(new_n145), .o1(new_n351));
  aoi113aa1n02x5               g256(.a(new_n125), .b(new_n121), .c(new_n118), .d(new_n112), .e(new_n119), .o1(new_n352));
  aoi022aa1n02x5               g257(.a(new_n123), .b(new_n125), .c(new_n351), .d(new_n352), .o1(\s[9] ));
endmodule


