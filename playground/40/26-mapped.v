// Benchmark "adder" written by ABC on Thu Jul 18 08:41:00 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n338, new_n340, new_n342, new_n344, new_n345, new_n346, new_n348,
    new_n349, new_n351, new_n352, new_n353, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\b[8] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\a[9] ), .b(new_n97), .out0(new_n98));
  nor002aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  inv000aa1n02x5               g004(.a(new_n99), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand22aa1n02x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aob012aa1n02x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  norp02aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n02x7               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor042aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nand42aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n02x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nand23aa1n03x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  tech160nm_fioai012aa1n05x5   g015(.a(new_n105), .b(new_n107), .c(new_n104), .o1(new_n111));
  nor042aa1n06x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand22aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor002aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nano23aa1n02x4               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  nor002aa1n04x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand22aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor002aa1d32x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nano23aa1n09x5               g025(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n121));
  nand02aa1n02x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  inv040aa1n02x5               g027(.a(new_n112), .o1(new_n123));
  aob012aa1n06x5               g028(.a(new_n123), .b(new_n114), .c(new_n113), .out0(new_n124));
  inv000aa1d42x5               g029(.a(new_n119), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[8] ), .b(\b[7] ), .c(new_n125), .o1(new_n126));
  aoi012aa1n12x5               g031(.a(new_n126), .b(new_n121), .c(new_n124), .o1(new_n127));
  aoai13aa1n12x5               g032(.a(new_n127), .b(new_n122), .c(new_n110), .d(new_n111), .o1(new_n128));
  xorc02aa1n03x5               g033(.a(\a[9] ), .b(\b[8] ), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n128), .b(new_n129), .o1(new_n130));
  xorc02aa1n03x5               g035(.a(\a[10] ), .b(\b[9] ), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n130), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g037(.a(\a[10] ), .o1(new_n133));
  xroi22aa1d04x5               g038(.a(new_n133), .b(\b[9] ), .c(new_n97), .d(\a[9] ), .out0(new_n134));
  oaoi03aa1n02x5               g039(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n135));
  norp02aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand42aa1n03x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n03x4               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoai13aa1n03x5               g043(.a(new_n138), .b(new_n135), .c(new_n128), .d(new_n134), .o1(new_n139));
  aoi112aa1n02x5               g044(.a(new_n138), .b(new_n135), .c(new_n128), .d(new_n134), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(\s[11] ));
  orn002aa1n02x5               g046(.a(\a[11] ), .b(\b[10] ), .o(new_n142));
  norp02aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n06x4               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n139), .c(new_n142), .out0(\s[12] ));
  aoi012aa1n02x5               g051(.a(new_n99), .b(new_n101), .c(new_n102), .o1(new_n147));
  nona23aa1n02x4               g052(.a(new_n108), .b(new_n105), .c(new_n104), .d(new_n107), .out0(new_n148));
  tech160nm_fioai012aa1n03p5x5 g053(.a(new_n111), .b(new_n148), .c(new_n147), .o1(new_n149));
  nanb02aa1n06x5               g054(.a(new_n122), .b(new_n149), .out0(new_n150));
  nano23aa1n09x5               g055(.a(new_n136), .b(new_n143), .c(new_n144), .d(new_n137), .out0(new_n151));
  inv040aa1n03x5               g056(.a(new_n151), .o1(new_n152));
  nano22aa1n03x7               g057(.a(new_n152), .b(new_n129), .c(new_n131), .out0(new_n153));
  inv000aa1n02x5               g058(.a(new_n153), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n143), .b(new_n136), .c(new_n144), .o1(new_n155));
  aobi12aa1n12x5               g060(.a(new_n155), .b(new_n151), .c(new_n135), .out0(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n154), .c(new_n150), .d(new_n127), .o1(new_n157));
  nor042aa1n03x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand42aa1n03x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  inv040aa1n03x5               g065(.a(new_n156), .o1(new_n161));
  aoi112aa1n02x5               g066(.a(new_n160), .b(new_n161), .c(new_n128), .d(new_n153), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n162), .b(new_n157), .c(new_n160), .o1(\s[13] ));
  orn002aa1n02x5               g068(.a(\a[13] ), .b(\b[12] ), .o(new_n164));
  aoai13aa1n02x5               g069(.a(new_n160), .b(new_n161), .c(new_n128), .d(new_n153), .o1(new_n165));
  nor002aa1n03x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand42aa1n06x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n165), .c(new_n164), .out0(\s[14] ));
  nano23aa1n06x5               g074(.a(new_n158), .b(new_n166), .c(new_n167), .d(new_n159), .out0(new_n170));
  aoai13aa1n03x5               g075(.a(new_n170), .b(new_n161), .c(new_n128), .d(new_n153), .o1(new_n171));
  aoi012aa1n09x5               g076(.a(new_n166), .b(new_n158), .c(new_n167), .o1(new_n172));
  nor042aa1n04x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  norb02aa1n03x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n171), .c(new_n172), .out0(\s[15] ));
  inv000aa1n02x5               g081(.a(new_n172), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n175), .b(new_n177), .c(new_n157), .d(new_n170), .o1(new_n178));
  nor002aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  aoib12aa1n02x5               g086(.a(new_n173), .b(new_n180), .c(new_n179), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n173), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n175), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n171), .d(new_n172), .o1(new_n185));
  aoi022aa1n02x7               g090(.a(new_n185), .b(new_n181), .c(new_n178), .d(new_n182), .o1(\s[16] ));
  nano23aa1n06x5               g091(.a(new_n173), .b(new_n179), .c(new_n180), .d(new_n174), .out0(new_n187));
  nanp02aa1n06x5               g092(.a(new_n187), .b(new_n170), .o1(new_n188));
  nano32aa1n09x5               g093(.a(new_n188), .b(new_n151), .c(new_n131), .d(new_n129), .out0(new_n189));
  nanp02aa1n02x5               g094(.a(new_n128), .b(new_n189), .o1(new_n190));
  inv000aa1n06x5               g095(.a(new_n189), .o1(new_n191));
  norp02aa1n02x5               g096(.a(\b[9] ), .b(\a[10] ), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n193));
  oai112aa1n03x5               g098(.a(new_n138), .b(new_n145), .c(new_n193), .d(new_n192), .o1(new_n194));
  tech160nm_fiaoi012aa1n04x5   g099(.a(new_n188), .b(new_n194), .c(new_n155), .o1(new_n195));
  oaoi03aa1n02x5               g100(.a(\a[16] ), .b(\b[15] ), .c(new_n183), .o1(new_n196));
  tech160nm_fiaoi012aa1n05x5   g101(.a(new_n196), .b(new_n187), .c(new_n177), .o1(new_n197));
  norb02aa1n03x5               g102(.a(new_n197), .b(new_n195), .out0(new_n198));
  aoai13aa1n12x5               g103(.a(new_n198), .b(new_n191), .c(new_n150), .d(new_n127), .o1(new_n199));
  tech160nm_fixorc02aa1n04x5   g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  norb03aa1n02x5               g105(.a(new_n197), .b(new_n195), .c(new_n200), .out0(new_n201));
  aoi022aa1n02x5               g106(.a(new_n199), .b(new_n200), .c(new_n190), .d(new_n201), .o1(\s[17] ));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(\b[16] ), .b(new_n203), .out0(new_n204));
  aoai13aa1n06x5               g109(.a(new_n197), .b(new_n188), .c(new_n194), .d(new_n155), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n200), .b(new_n205), .c(new_n128), .d(new_n189), .o1(new_n206));
  tech160nm_fixorc02aa1n05x5   g111(.a(\a[18] ), .b(\b[17] ), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n206), .c(new_n204), .out0(\s[18] ));
  and002aa1n02x5               g113(.a(new_n207), .b(new_n200), .o(new_n209));
  aoai13aa1n04x5               g114(.a(new_n209), .b(new_n205), .c(new_n128), .d(new_n189), .o1(new_n210));
  norp02aa1n02x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  aoi112aa1n09x5               g116(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n212));
  nor002aa1n02x5               g117(.a(new_n212), .b(new_n211), .o1(new_n213));
  xorc02aa1n12x5               g118(.a(\a[19] ), .b(\b[18] ), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n210), .c(new_n213), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g121(.a(new_n213), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n214), .b(new_n217), .c(new_n199), .d(new_n209), .o1(new_n218));
  tech160nm_fixorc02aa1n04x5   g123(.a(\a[20] ), .b(\b[19] ), .out0(new_n219));
  nor042aa1n04x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  norp02aa1n02x5               g125(.a(new_n219), .b(new_n220), .o1(new_n221));
  inv000aa1n02x5               g126(.a(new_n220), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n214), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n222), .b(new_n223), .c(new_n210), .d(new_n213), .o1(new_n224));
  aoi022aa1n02x5               g129(.a(new_n224), .b(new_n219), .c(new_n218), .d(new_n221), .o1(\s[20] ));
  nano32aa1n02x4               g130(.a(new_n223), .b(new_n219), .c(new_n200), .d(new_n207), .out0(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n205), .c(new_n128), .d(new_n189), .o1(new_n227));
  oai112aa1n06x5               g132(.a(new_n214), .b(new_n219), .c(new_n212), .d(new_n211), .o1(new_n228));
  oao003aa1n06x5               g133(.a(\a[20] ), .b(\b[19] ), .c(new_n222), .carry(new_n229));
  nanp02aa1n06x5               g134(.a(new_n228), .b(new_n229), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nor002aa1d32x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nand42aa1d28x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  xnbna2aa1n03x5               g139(.a(new_n234), .b(new_n227), .c(new_n231), .out0(\s[21] ));
  aoai13aa1n03x5               g140(.a(new_n234), .b(new_n230), .c(new_n199), .d(new_n226), .o1(new_n236));
  nor042aa1n06x5               g141(.a(\b[21] ), .b(\a[22] ), .o1(new_n237));
  nanp02aa1n24x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  aoib12aa1n02x5               g144(.a(new_n232), .b(new_n238), .c(new_n237), .out0(new_n240));
  inv000aa1n06x5               g145(.a(new_n232), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n234), .o1(new_n242));
  aoai13aa1n02x5               g147(.a(new_n241), .b(new_n242), .c(new_n227), .d(new_n231), .o1(new_n243));
  aoi022aa1n02x7               g148(.a(new_n243), .b(new_n239), .c(new_n236), .d(new_n240), .o1(\s[22] ));
  inv000aa1d42x5               g149(.a(\a[19] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(\a[20] ), .o1(new_n246));
  xroi22aa1d04x5               g151(.a(new_n245), .b(\b[18] ), .c(new_n246), .d(\b[19] ), .out0(new_n247));
  nano23aa1d15x5               g152(.a(new_n232), .b(new_n237), .c(new_n238), .d(new_n233), .out0(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  nano32aa1n02x4               g154(.a(new_n249), .b(new_n247), .c(new_n207), .d(new_n200), .out0(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n205), .c(new_n128), .d(new_n189), .o1(new_n251));
  oaoi03aa1n06x5               g156(.a(\a[22] ), .b(\b[21] ), .c(new_n241), .o1(new_n252));
  aoi012aa1n03x5               g157(.a(new_n252), .b(new_n230), .c(new_n248), .o1(new_n253));
  nanp02aa1n02x5               g158(.a(new_n251), .b(new_n253), .o1(new_n254));
  nor042aa1n09x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  nanp02aa1n03x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  aoi112aa1n02x5               g162(.a(new_n257), .b(new_n252), .c(new_n230), .d(new_n248), .o1(new_n258));
  aoi022aa1n02x5               g163(.a(new_n254), .b(new_n257), .c(new_n251), .d(new_n258), .o1(\s[23] ));
  inv000aa1n02x5               g164(.a(new_n253), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n257), .b(new_n260), .c(new_n199), .d(new_n250), .o1(new_n261));
  nor042aa1n02x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  nand02aa1n03x5               g167(.a(\b[23] ), .b(\a[24] ), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n263), .b(new_n262), .out0(new_n264));
  aoib12aa1n02x5               g169(.a(new_n255), .b(new_n263), .c(new_n262), .out0(new_n265));
  inv000aa1n02x5               g170(.a(new_n255), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n257), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n266), .b(new_n267), .c(new_n251), .d(new_n253), .o1(new_n268));
  aoi022aa1n02x7               g173(.a(new_n268), .b(new_n264), .c(new_n261), .d(new_n265), .o1(\s[24] ));
  nano23aa1n06x5               g174(.a(new_n255), .b(new_n262), .c(new_n263), .d(new_n256), .out0(new_n270));
  nand22aa1n03x5               g175(.a(new_n270), .b(new_n248), .o1(new_n271));
  nano32aa1n02x4               g176(.a(new_n271), .b(new_n247), .c(new_n207), .d(new_n200), .out0(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n205), .c(new_n128), .d(new_n189), .o1(new_n273));
  oaoi03aa1n02x5               g178(.a(\a[24] ), .b(\b[23] ), .c(new_n266), .o1(new_n274));
  tech160nm_fiaoi012aa1n03p5x5 g179(.a(new_n274), .b(new_n270), .c(new_n252), .o1(new_n275));
  aoai13aa1n12x5               g180(.a(new_n275), .b(new_n271), .c(new_n228), .d(new_n229), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  nanp02aa1n02x5               g182(.a(new_n273), .b(new_n277), .o1(new_n278));
  xorc02aa1n12x5               g183(.a(\a[25] ), .b(\b[24] ), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  oai112aa1n02x5               g185(.a(new_n275), .b(new_n280), .c(new_n231), .d(new_n271), .o1(new_n281));
  aboi22aa1n03x5               g186(.a(new_n281), .b(new_n273), .c(new_n278), .d(new_n279), .out0(\s[25] ));
  aoai13aa1n02x7               g187(.a(new_n279), .b(new_n276), .c(new_n199), .d(new_n272), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .out0(new_n284));
  nor042aa1n03x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n284), .b(new_n285), .o1(new_n286));
  inv040aa1n03x5               g191(.a(new_n285), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n280), .c(new_n273), .d(new_n277), .o1(new_n288));
  aoi022aa1n02x7               g193(.a(new_n288), .b(new_n284), .c(new_n283), .d(new_n286), .o1(\s[26] ));
  and002aa1n02x5               g194(.a(new_n284), .b(new_n279), .o(new_n290));
  nano32aa1n03x7               g195(.a(new_n271), .b(new_n290), .c(new_n209), .d(new_n247), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n205), .c(new_n128), .d(new_n189), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n293));
  inv000aa1n02x5               g198(.a(new_n293), .o1(new_n294));
  tech160nm_fiaoi012aa1n05x5   g199(.a(new_n294), .b(new_n276), .c(new_n290), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(new_n292), .b(new_n295), .o1(new_n296));
  xorc02aa1n12x5               g201(.a(\a[27] ), .b(\b[26] ), .out0(new_n297));
  aoi112aa1n02x5               g202(.a(new_n297), .b(new_n294), .c(new_n276), .d(new_n290), .o1(new_n298));
  aoi022aa1n02x5               g203(.a(new_n296), .b(new_n297), .c(new_n292), .d(new_n298), .o1(\s[27] ));
  aob012aa1n06x5               g204(.a(new_n293), .b(new_n276), .c(new_n290), .out0(new_n300));
  aoai13aa1n03x5               g205(.a(new_n297), .b(new_n300), .c(new_n199), .d(new_n291), .o1(new_n301));
  tech160nm_fixorc02aa1n02p5x5 g206(.a(\a[28] ), .b(\b[27] ), .out0(new_n302));
  norp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n302), .b(new_n303), .o1(new_n304));
  inv000aa1n06x5               g209(.a(new_n303), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n297), .o1(new_n306));
  aoai13aa1n02x7               g211(.a(new_n305), .b(new_n306), .c(new_n292), .d(new_n295), .o1(new_n307));
  aoi022aa1n03x5               g212(.a(new_n307), .b(new_n302), .c(new_n301), .d(new_n304), .o1(\s[28] ));
  and002aa1n02x5               g213(.a(new_n302), .b(new_n297), .o(new_n309));
  aoai13aa1n06x5               g214(.a(new_n309), .b(new_n300), .c(new_n199), .d(new_n291), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n309), .o1(new_n311));
  oaoi03aa1n12x5               g216(.a(\a[28] ), .b(\b[27] ), .c(new_n305), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n312), .o1(new_n313));
  aoai13aa1n02x7               g218(.a(new_n313), .b(new_n311), .c(new_n292), .d(new_n295), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .out0(new_n315));
  norp02aa1n02x5               g220(.a(new_n312), .b(new_n315), .o1(new_n316));
  aoi022aa1n03x5               g221(.a(new_n314), .b(new_n315), .c(new_n310), .d(new_n316), .o1(\s[29] ));
  xorb03aa1n02x5               g222(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g223(.a(new_n306), .b(new_n302), .c(new_n315), .out0(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n300), .c(new_n199), .d(new_n291), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n319), .o1(new_n321));
  inv000aa1d42x5               g226(.a(\a[29] ), .o1(new_n322));
  inv000aa1d42x5               g227(.a(\b[28] ), .o1(new_n323));
  oaoi03aa1n02x5               g228(.a(new_n322), .b(new_n323), .c(new_n312), .o1(new_n324));
  aoai13aa1n02x7               g229(.a(new_n324), .b(new_n321), .c(new_n292), .d(new_n295), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .out0(new_n326));
  oabi12aa1n02x5               g231(.a(new_n326), .b(\a[29] ), .c(\b[28] ), .out0(new_n327));
  oaoi13aa1n02x5               g232(.a(new_n327), .b(new_n312), .c(new_n322), .d(new_n323), .o1(new_n328));
  aoi022aa1n03x5               g233(.a(new_n325), .b(new_n326), .c(new_n320), .d(new_n328), .o1(\s[30] ));
  xorc02aa1n02x5               g234(.a(\a[31] ), .b(\b[30] ), .out0(new_n330));
  nano32aa1n06x5               g235(.a(new_n306), .b(new_n326), .c(new_n302), .d(new_n315), .out0(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n300), .c(new_n199), .d(new_n291), .o1(new_n332));
  inv000aa1d42x5               g237(.a(new_n331), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n324), .carry(new_n334));
  aoai13aa1n02x7               g239(.a(new_n334), .b(new_n333), .c(new_n292), .d(new_n295), .o1(new_n335));
  norb02aa1n02x5               g240(.a(new_n334), .b(new_n330), .out0(new_n336));
  aoi022aa1n03x5               g241(.a(new_n335), .b(new_n330), .c(new_n332), .d(new_n336), .o1(\s[31] ));
  aoi112aa1n02x5               g242(.a(new_n109), .b(new_n99), .c(new_n101), .d(new_n102), .o1(new_n338));
  aoi012aa1n02x5               g243(.a(new_n338), .b(new_n103), .c(new_n109), .o1(\s[3] ));
  aoi112aa1n02x5               g244(.a(new_n107), .b(new_n106), .c(new_n103), .d(new_n108), .o1(new_n340));
  aoib12aa1n02x5               g245(.a(new_n340), .b(new_n149), .c(new_n104), .out0(\s[4] ));
  norb02aa1n02x5               g246(.a(new_n115), .b(new_n114), .out0(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n342), .b(new_n110), .c(new_n111), .out0(\s[5] ));
  norb02aa1n02x5               g248(.a(new_n113), .b(new_n112), .out0(new_n344));
  aoai13aa1n02x5               g249(.a(new_n344), .b(new_n114), .c(new_n149), .d(new_n115), .o1(new_n345));
  aoi112aa1n02x5               g250(.a(new_n114), .b(new_n344), .c(new_n149), .d(new_n342), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n345), .b(new_n346), .out0(\s[6] ));
  nanb02aa1n02x5               g252(.a(new_n119), .b(new_n120), .out0(new_n348));
  inv000aa1d42x5               g253(.a(new_n348), .o1(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n349), .b(new_n345), .c(new_n123), .out0(\s[7] ));
  nanb02aa1n02x5               g255(.a(new_n117), .b(new_n118), .out0(new_n351));
  aob012aa1n02x5               g256(.a(new_n349), .b(new_n345), .c(new_n123), .out0(new_n352));
  aoai13aa1n02x5               g257(.a(new_n125), .b(new_n348), .c(new_n345), .d(new_n123), .o1(new_n353));
  aoib12aa1n02x5               g258(.a(new_n119), .b(new_n118), .c(new_n117), .out0(new_n354));
  aboi22aa1n03x5               g259(.a(new_n351), .b(new_n353), .c(new_n352), .d(new_n354), .out0(\s[8] ));
  xnbna2aa1n03x5               g260(.a(new_n129), .b(new_n150), .c(new_n127), .out0(\s[9] ));
endmodule


