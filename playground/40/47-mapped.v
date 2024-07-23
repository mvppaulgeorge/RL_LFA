// Benchmark "adder" written by ABC on Thu Jul 18 08:53:58 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
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
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n339, new_n341, new_n342, new_n345, new_n346, new_n349,
    new_n350, new_n351, new_n353, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  orn002aa1n24x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nand42aa1n20x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n06x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aob012aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nor002aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n02x7               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  xorc02aa1n12x5               g010(.a(\a[3] ), .b(\b[2] ), .out0(new_n106));
  nanp03aa1n06x5               g011(.a(new_n102), .b(new_n106), .c(new_n105), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\a[3] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[2] ), .o1(new_n109));
  aoai13aa1n06x5               g014(.a(new_n104), .b(new_n103), .c(new_n108), .d(new_n109), .o1(new_n110));
  nanp02aa1n06x5               g015(.a(new_n107), .b(new_n110), .o1(new_n111));
  xnrc02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .out0(new_n112));
  xorc02aa1n12x5               g017(.a(\a[7] ), .b(\b[6] ), .out0(new_n113));
  nor022aa1n08x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand42aa1n04x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  tech160nm_fixorc02aa1n05x5   g021(.a(\a[5] ), .b(\b[4] ), .out0(new_n117));
  nano32aa1n03x7               g022(.a(new_n112), .b(new_n117), .c(new_n113), .d(new_n116), .out0(new_n118));
  xorc02aa1n12x5               g023(.a(\a[8] ), .b(\b[7] ), .out0(new_n119));
  inv000aa1n06x5               g024(.a(new_n114), .o1(new_n120));
  nor042aa1n09x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  aob012aa1d15x5               g026(.a(new_n120), .b(new_n121), .c(new_n115), .out0(new_n122));
  orn002aa1n24x5               g027(.a(\a[7] ), .b(\b[6] ), .o(new_n123));
  oaoi03aa1n09x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  aoi013aa1n09x5               g029(.a(new_n124), .b(new_n122), .c(new_n113), .d(new_n119), .o1(new_n125));
  inv000aa1n02x5               g030(.a(new_n125), .o1(new_n126));
  xnrc02aa1n12x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n126), .c(new_n111), .d(new_n118), .o1(new_n129));
  xorc02aa1n12x5               g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n129), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g036(.a(\a[10] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(\b[9] ), .o1(new_n133));
  aboi22aa1d24x5               g038(.a(\b[8] ), .b(new_n97), .c(new_n133), .d(new_n132), .out0(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norp02aa1n24x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  oai112aa1n02x5               g042(.a(new_n137), .b(new_n135), .c(new_n133), .d(new_n132), .o1(new_n138));
  aoi012aa1n02x7               g043(.a(new_n138), .b(new_n129), .c(new_n134), .o1(new_n139));
  nanb02aa1n06x5               g044(.a(new_n136), .b(new_n135), .out0(new_n140));
  aoi022aa1n02x5               g045(.a(new_n129), .b(new_n134), .c(\b[9] ), .d(\a[10] ), .o1(new_n141));
  aoib12aa1n02x5               g046(.a(new_n139), .b(new_n140), .c(new_n141), .out0(\s[11] ));
  aoai13aa1n02x7               g047(.a(new_n137), .b(new_n138), .c(new_n129), .d(new_n134), .o1(new_n143));
  nor042aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  and002aa1n12x5               g049(.a(\b[11] ), .b(\a[12] ), .o(new_n145));
  nor042aa1n03x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  oab012aa1n02x4               g051(.a(new_n136), .b(new_n145), .c(new_n144), .out0(new_n147));
  aboi22aa1n03x5               g052(.a(new_n139), .b(new_n147), .c(new_n143), .d(new_n146), .out0(\s[12] ));
  nona23aa1d18x5               g053(.a(new_n146), .b(new_n130), .c(new_n127), .d(new_n140), .out0(new_n149));
  inv040aa1n02x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n126), .c(new_n111), .d(new_n118), .o1(new_n151));
  oai022aa1n02x5               g056(.a(new_n132), .b(new_n133), .c(\b[10] ), .d(\a[11] ), .o1(new_n152));
  aoi112aa1n03x5               g057(.a(new_n145), .b(new_n144), .c(\a[11] ), .d(\b[10] ), .o1(new_n153));
  nona22aa1n03x5               g058(.a(new_n153), .b(new_n134), .c(new_n152), .out0(new_n154));
  oabi12aa1n02x7               g059(.a(new_n145), .b(new_n136), .c(new_n144), .out0(new_n155));
  nand22aa1n03x5               g060(.a(new_n154), .b(new_n155), .o1(new_n156));
  nanb02aa1n03x5               g061(.a(new_n156), .b(new_n151), .out0(new_n157));
  xorc02aa1n02x5               g062(.a(\a[13] ), .b(\b[12] ), .out0(new_n158));
  nano22aa1n02x4               g063(.a(new_n158), .b(new_n154), .c(new_n155), .out0(new_n159));
  aoi022aa1n02x5               g064(.a(new_n157), .b(new_n158), .c(new_n151), .d(new_n159), .o1(\s[13] ));
  inv040aa1d28x5               g065(.a(\a[13] ), .o1(new_n161));
  nanb02aa1d24x5               g066(.a(\b[12] ), .b(new_n161), .out0(new_n162));
  xnrc02aa1n02x5               g067(.a(\b[6] ), .b(\a[7] ), .out0(new_n163));
  nona23aa1n03x5               g068(.a(new_n116), .b(new_n117), .c(new_n163), .d(new_n112), .out0(new_n164));
  aoai13aa1n06x5               g069(.a(new_n125), .b(new_n164), .c(new_n107), .d(new_n110), .o1(new_n165));
  aoai13aa1n03x5               g070(.a(new_n158), .b(new_n156), .c(new_n165), .d(new_n150), .o1(new_n166));
  xorc02aa1n02x5               g071(.a(\a[14] ), .b(\b[13] ), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n166), .c(new_n162), .out0(\s[14] ));
  inv000aa1n06x5               g073(.a(\a[14] ), .o1(new_n169));
  xroi22aa1d06x4               g074(.a(new_n161), .b(\b[12] ), .c(new_n169), .d(\b[13] ), .out0(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n156), .c(new_n165), .d(new_n150), .o1(new_n171));
  oaoi03aa1n12x5               g076(.a(\a[14] ), .b(\b[13] ), .c(new_n162), .o1(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  xorc02aa1n12x5               g078(.a(\a[15] ), .b(\b[14] ), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n171), .c(new_n173), .out0(\s[15] ));
  aoai13aa1n03x5               g080(.a(new_n174), .b(new_n172), .c(new_n157), .d(new_n170), .o1(new_n176));
  nor042aa1d18x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  inv000aa1d42x5               g083(.a(new_n174), .o1(new_n179));
  aoai13aa1n02x5               g084(.a(new_n178), .b(new_n179), .c(new_n171), .d(new_n173), .o1(new_n180));
  xorc02aa1n02x5               g085(.a(\a[16] ), .b(\b[15] ), .out0(new_n181));
  norp02aa1n02x5               g086(.a(new_n181), .b(new_n177), .o1(new_n182));
  aoi022aa1n02x5               g087(.a(new_n180), .b(new_n181), .c(new_n176), .d(new_n182), .o1(\s[16] ));
  nanp02aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  tech160nm_fixnrc02aa1n03p5x5 g089(.a(\b[15] ), .b(\a[16] ), .out0(new_n185));
  nano22aa1n03x7               g090(.a(new_n185), .b(new_n178), .c(new_n184), .out0(new_n186));
  nano22aa1d15x5               g091(.a(new_n149), .b(new_n170), .c(new_n186), .out0(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n126), .c(new_n111), .d(new_n118), .o1(new_n188));
  nand03aa1n02x5               g093(.a(new_n172), .b(new_n174), .c(new_n181), .o1(new_n189));
  oao003aa1n02x5               g094(.a(\a[16] ), .b(\b[15] ), .c(new_n178), .carry(new_n190));
  nanp02aa1n02x5               g095(.a(new_n189), .b(new_n190), .o1(new_n191));
  aoi013aa1n06x4               g096(.a(new_n191), .b(new_n156), .c(new_n170), .d(new_n186), .o1(new_n192));
  nanp02aa1n09x5               g097(.a(new_n188), .b(new_n192), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  nanb03aa1n02x5               g099(.a(new_n194), .b(new_n189), .c(new_n190), .out0(new_n195));
  aoi013aa1n02x4               g100(.a(new_n195), .b(new_n156), .c(new_n170), .d(new_n186), .o1(new_n196));
  aoi022aa1n02x5               g101(.a(new_n193), .b(new_n194), .c(new_n188), .d(new_n196), .o1(\s[17] ));
  inv040aa1d32x5               g102(.a(\a[17] ), .o1(new_n198));
  inv040aa1d28x5               g103(.a(\b[16] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(new_n199), .b(new_n198), .o1(new_n200));
  nand02aa1n02x5               g105(.a(new_n170), .b(new_n186), .o1(new_n201));
  aobi12aa1n03x7               g106(.a(new_n190), .b(new_n186), .c(new_n172), .out0(new_n202));
  aoai13aa1n04x5               g107(.a(new_n202), .b(new_n201), .c(new_n154), .d(new_n155), .o1(new_n203));
  aoai13aa1n03x5               g108(.a(new_n194), .b(new_n203), .c(new_n165), .d(new_n187), .o1(new_n204));
  nor022aa1n08x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand22aa1n06x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  norb02aa1n06x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n204), .c(new_n200), .out0(\s[18] ));
  and002aa1n02x5               g113(.a(new_n194), .b(new_n207), .o(new_n209));
  aoai13aa1n03x5               g114(.a(new_n209), .b(new_n203), .c(new_n165), .d(new_n187), .o1(new_n210));
  aoi013aa1n09x5               g115(.a(new_n205), .b(new_n206), .c(new_n198), .d(new_n199), .o1(new_n211));
  nor002aa1d32x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanp02aa1n12x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n210), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n12x5               g121(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n214), .b(new_n217), .c(new_n193), .d(new_n209), .o1(new_n218));
  inv040aa1n08x5               g123(.a(new_n212), .o1(new_n219));
  inv000aa1n02x5               g124(.a(new_n214), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n220), .c(new_n210), .d(new_n211), .o1(new_n221));
  nor022aa1n16x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  nanp02aa1n12x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  inv000aa1d42x5               g129(.a(\a[19] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\b[18] ), .o1(new_n226));
  aboi22aa1n03x5               g131(.a(new_n222), .b(new_n223), .c(new_n225), .d(new_n226), .out0(new_n227));
  aoi022aa1n03x5               g132(.a(new_n221), .b(new_n224), .c(new_n218), .d(new_n227), .o1(\s[20] ));
  nano23aa1n06x5               g133(.a(new_n212), .b(new_n222), .c(new_n223), .d(new_n213), .out0(new_n229));
  nand23aa1d12x5               g134(.a(new_n229), .b(new_n194), .c(new_n207), .o1(new_n230));
  aoi012aa1n06x5               g135(.a(new_n230), .b(new_n188), .c(new_n192), .o1(new_n231));
  nona23aa1d16x5               g136(.a(new_n223), .b(new_n213), .c(new_n212), .d(new_n222), .out0(new_n232));
  oaoi03aa1n09x5               g137(.a(\a[20] ), .b(\b[19] ), .c(new_n219), .o1(new_n233));
  inv030aa1n03x5               g138(.a(new_n233), .o1(new_n234));
  oai012aa1d24x5               g139(.a(new_n234), .b(new_n232), .c(new_n211), .o1(new_n235));
  xnrc02aa1n06x5               g140(.a(\b[20] ), .b(\a[21] ), .out0(new_n236));
  oabi12aa1n03x5               g141(.a(new_n236), .b(new_n231), .c(new_n235), .out0(new_n237));
  oai112aa1n02x5               g142(.a(new_n234), .b(new_n236), .c(new_n232), .d(new_n211), .o1(new_n238));
  oa0012aa1n03x5               g143(.a(new_n237), .b(new_n231), .c(new_n238), .o(\s[21] ));
  inv000aa1d42x5               g144(.a(new_n230), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n240), .b(new_n203), .c(new_n165), .d(new_n187), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n235), .o1(new_n242));
  nor042aa1n06x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  inv000aa1n03x5               g148(.a(new_n243), .o1(new_n244));
  aoai13aa1n02x7               g149(.a(new_n244), .b(new_n236), .c(new_n241), .d(new_n242), .o1(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[21] ), .b(\a[22] ), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n246), .b(new_n243), .out0(new_n248));
  aoi022aa1n03x5               g153(.a(new_n245), .b(new_n247), .c(new_n237), .d(new_n248), .o1(\s[22] ));
  nor042aa1n06x5               g154(.a(new_n246), .b(new_n236), .o1(new_n250));
  norb02aa1n03x5               g155(.a(new_n250), .b(new_n230), .out0(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n203), .c(new_n165), .d(new_n187), .o1(new_n252));
  oao003aa1n12x5               g157(.a(\a[22] ), .b(\b[21] ), .c(new_n244), .carry(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  tech160nm_fiaoi012aa1n05x5   g159(.a(new_n254), .b(new_n235), .c(new_n250), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  xorc02aa1n12x5               g161(.a(\a[23] ), .b(\b[22] ), .out0(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n256), .c(new_n193), .d(new_n251), .o1(new_n258));
  aoi112aa1n02x5               g163(.a(new_n257), .b(new_n254), .c(new_n235), .d(new_n250), .o1(new_n259));
  aobi12aa1n02x7               g164(.a(new_n258), .b(new_n259), .c(new_n252), .out0(\s[23] ));
  nor042aa1n09x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n257), .o1(new_n263));
  aoai13aa1n02x7               g168(.a(new_n262), .b(new_n263), .c(new_n252), .d(new_n255), .o1(new_n264));
  tech160nm_fixorc02aa1n03p5x5 g169(.a(\a[24] ), .b(\b[23] ), .out0(new_n265));
  norp02aa1n02x5               g170(.a(new_n265), .b(new_n261), .o1(new_n266));
  aoi022aa1n03x5               g171(.a(new_n264), .b(new_n265), .c(new_n258), .d(new_n266), .o1(\s[24] ));
  nano32aa1n03x7               g172(.a(new_n230), .b(new_n265), .c(new_n250), .d(new_n257), .out0(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n203), .c(new_n165), .d(new_n187), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n250), .b(new_n233), .c(new_n229), .d(new_n217), .o1(new_n270));
  and002aa1n06x5               g175(.a(new_n265), .b(new_n257), .o(new_n271));
  inv020aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[24] ), .b(\b[23] ), .c(new_n262), .carry(new_n273));
  aoai13aa1n04x5               g178(.a(new_n273), .b(new_n272), .c(new_n270), .d(new_n253), .o1(new_n274));
  xorc02aa1n12x5               g179(.a(\a[25] ), .b(\b[24] ), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n274), .c(new_n193), .d(new_n268), .o1(new_n276));
  aoai13aa1n04x5               g181(.a(new_n271), .b(new_n254), .c(new_n235), .d(new_n250), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n275), .o1(new_n278));
  and003aa1n02x5               g183(.a(new_n277), .b(new_n278), .c(new_n273), .o(new_n279));
  aobi12aa1n02x7               g184(.a(new_n276), .b(new_n279), .c(new_n269), .out0(\s[25] ));
  inv000aa1n02x5               g185(.a(new_n274), .o1(new_n281));
  norp02aa1n02x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  aoai13aa1n02x7               g188(.a(new_n283), .b(new_n278), .c(new_n269), .d(new_n281), .o1(new_n284));
  xorc02aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  norp02aa1n02x5               g190(.a(new_n285), .b(new_n282), .o1(new_n286));
  aoi022aa1n03x5               g191(.a(new_n284), .b(new_n285), .c(new_n276), .d(new_n286), .o1(\s[26] ));
  and002aa1n03x5               g192(.a(new_n285), .b(new_n275), .o(new_n288));
  inv020aa1n03x5               g193(.a(new_n288), .o1(new_n289));
  nano23aa1n06x5               g194(.a(new_n289), .b(new_n230), .c(new_n271), .d(new_n250), .out0(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n203), .c(new_n165), .d(new_n187), .o1(new_n291));
  nanp02aa1n02x5               g196(.a(\b[25] ), .b(\a[26] ), .o1(new_n292));
  oai022aa1n02x5               g197(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(new_n293), .b(new_n292), .o1(new_n294));
  aoai13aa1n04x5               g199(.a(new_n294), .b(new_n289), .c(new_n277), .d(new_n273), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n295), .c(new_n193), .d(new_n290), .o1(new_n297));
  aoi122aa1n02x5               g202(.a(new_n296), .b(new_n292), .c(new_n293), .d(new_n274), .e(new_n288), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n297), .b(new_n298), .c(new_n291), .out0(\s[27] ));
  aoi022aa1n06x5               g204(.a(new_n274), .b(new_n288), .c(new_n292), .d(new_n293), .o1(new_n300));
  nor042aa1d18x5               g205(.a(\b[26] ), .b(\a[27] ), .o1(new_n301));
  inv040aa1n08x5               g206(.a(new_n301), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n296), .o1(new_n303));
  aoai13aa1n02x7               g208(.a(new_n302), .b(new_n303), .c(new_n300), .d(new_n291), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[28] ), .b(\b[27] ), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n301), .o1(new_n306));
  aoi022aa1n02x7               g211(.a(new_n304), .b(new_n305), .c(new_n297), .d(new_n306), .o1(\s[28] ));
  and002aa1n02x5               g212(.a(new_n305), .b(new_n296), .o(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n295), .c(new_n193), .d(new_n290), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  oao003aa1n09x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n311));
  aoai13aa1n02x7               g216(.a(new_n311), .b(new_n310), .c(new_n300), .d(new_n291), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n311), .b(new_n313), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n309), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g221(.a(new_n303), .b(new_n305), .c(new_n313), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n295), .c(new_n193), .d(new_n290), .o1(new_n318));
  inv000aa1n02x5               g223(.a(new_n317), .o1(new_n319));
  tech160nm_fioaoi03aa1n03p5x5 g224(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n320), .o1(new_n321));
  aoai13aa1n02x7               g226(.a(new_n321), .b(new_n319), .c(new_n300), .d(new_n291), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .out0(new_n323));
  and002aa1n02x5               g228(.a(\b[28] ), .b(\a[29] ), .o(new_n324));
  oabi12aa1n02x5               g229(.a(new_n323), .b(\a[29] ), .c(\b[28] ), .out0(new_n325));
  oab012aa1n02x4               g230(.a(new_n325), .b(new_n311), .c(new_n324), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n322), .b(new_n323), .c(new_n318), .d(new_n326), .o1(\s[30] ));
  nano32aa1n03x7               g232(.a(new_n303), .b(new_n323), .c(new_n305), .d(new_n313), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n295), .c(new_n193), .d(new_n290), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[31] ), .b(\b[30] ), .out0(new_n330));
  inv000aa1d42x5               g235(.a(\a[30] ), .o1(new_n331));
  inv000aa1d42x5               g236(.a(\b[29] ), .o1(new_n332));
  oabi12aa1n02x5               g237(.a(new_n330), .b(\a[30] ), .c(\b[29] ), .out0(new_n333));
  oaoi13aa1n03x5               g238(.a(new_n333), .b(new_n320), .c(new_n331), .d(new_n332), .o1(new_n334));
  inv000aa1d42x5               g239(.a(new_n328), .o1(new_n335));
  oaoi03aa1n02x5               g240(.a(new_n331), .b(new_n332), .c(new_n320), .o1(new_n336));
  aoai13aa1n02x7               g241(.a(new_n336), .b(new_n335), .c(new_n300), .d(new_n291), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n337), .b(new_n330), .c(new_n329), .d(new_n334), .o1(\s[31] ));
  nanp03aa1n02x5               g243(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n106), .b(new_n339), .c(new_n99), .out0(\s[3] ));
  obai22aa1n02x7               g245(.a(new_n104), .b(new_n103), .c(\a[3] ), .d(\b[2] ), .out0(new_n341));
  aoi012aa1n02x5               g246(.a(new_n341), .b(new_n102), .c(new_n106), .o1(new_n342));
  oaoi13aa1n02x5               g247(.a(new_n342), .b(new_n111), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g248(.a(new_n117), .b(new_n107), .c(new_n110), .out0(\s[5] ));
  aoai13aa1n02x5               g249(.a(new_n116), .b(new_n121), .c(new_n111), .d(new_n117), .o1(new_n345));
  aoi112aa1n02x5               g250(.a(new_n121), .b(new_n116), .c(new_n111), .d(new_n117), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n345), .b(new_n346), .out0(\s[6] ));
  xnbna2aa1n03x5               g252(.a(new_n113), .b(new_n345), .c(new_n120), .out0(\s[7] ));
  aob012aa1n02x5               g253(.a(new_n113), .b(new_n345), .c(new_n120), .out0(new_n349));
  aoai13aa1n02x5               g254(.a(new_n123), .b(new_n163), .c(new_n345), .d(new_n120), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n123), .b(new_n119), .out0(new_n351));
  aoi022aa1n03x5               g256(.a(new_n350), .b(new_n119), .c(new_n349), .d(new_n351), .o1(\s[8] ));
  nanp02aa1n02x5               g257(.a(new_n111), .b(new_n118), .o1(new_n353));
  aoi113aa1n02x5               g258(.a(new_n128), .b(new_n124), .c(new_n122), .d(new_n113), .e(new_n119), .o1(new_n354));
  aoi022aa1n02x5               g259(.a(new_n165), .b(new_n128), .c(new_n353), .d(new_n354), .o1(\s[9] ));
endmodule


