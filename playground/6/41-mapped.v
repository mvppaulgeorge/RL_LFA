// Benchmark "adder" written by ABC on Wed Jul 17 15:22:39 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n341, new_n342, new_n343, new_n344,
    new_n346, new_n347, new_n350, new_n351, new_n352, new_n354, new_n355,
    new_n357;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n09x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  nand02aa1d10x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nor042aa1n03x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand22aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano23aa1n06x5               g009(.a(new_n103), .b(new_n102), .c(new_n104), .d(new_n101), .out0(new_n105));
  nanp02aa1n03x5               g010(.a(new_n105), .b(new_n100), .o1(new_n106));
  tech160nm_fiaoi012aa1n05x5   g011(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n107));
  nand02aa1d04x5               g012(.a(new_n106), .b(new_n107), .o1(new_n108));
  nand42aa1n06x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor002aa1n03x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor002aa1n16x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n12x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n06x5               g017(.a(new_n111), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n113));
  xorc02aa1n12x5               g018(.a(\a[5] ), .b(\b[4] ), .out0(new_n114));
  xorc02aa1n12x5               g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  nand23aa1d12x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  inv000aa1n02x5               g021(.a(new_n116), .o1(new_n117));
  and002aa1n06x5               g022(.a(\b[5] ), .b(\a[6] ), .o(new_n118));
  inv020aa1n02x5               g023(.a(new_n118), .o1(new_n119));
  oai022aa1d24x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  aoi013aa1n02x4               g025(.a(new_n111), .b(new_n119), .c(new_n120), .d(new_n112), .o1(new_n121));
  oaoi03aa1n06x5               g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  xorc02aa1n12x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n117), .d(new_n108), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  aobi12aa1n09x5               g031(.a(new_n107), .b(new_n105), .c(new_n100), .out0(new_n127));
  inv000aa1n02x5               g032(.a(new_n111), .o1(new_n128));
  inv000aa1d42x5               g033(.a(\a[6] ), .o1(new_n129));
  inv000aa1d42x5               g034(.a(\b[5] ), .o1(new_n130));
  oai112aa1n03x5               g035(.a(new_n120), .b(new_n112), .c(new_n130), .d(new_n129), .o1(new_n131));
  nand02aa1n02x5               g036(.a(new_n131), .b(new_n128), .o1(new_n132));
  tech160nm_fiaoi012aa1n04x5   g037(.a(new_n110), .b(new_n132), .c(new_n109), .o1(new_n133));
  oai012aa1n18x5               g038(.a(new_n133), .b(new_n127), .c(new_n116), .o1(new_n134));
  aoai13aa1n06x5               g039(.a(new_n125), .b(new_n97), .c(new_n134), .d(new_n123), .o1(new_n135));
  nor022aa1n16x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nanp02aa1n04x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  oai012aa1n18x5               g042(.a(new_n137), .b(new_n136), .c(new_n97), .o1(new_n138));
  nor042aa1d18x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand42aa1n20x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n135), .c(new_n138), .out0(\s[11] ));
  nand42aa1n02x5               g047(.a(new_n124), .b(new_n98), .o1(new_n143));
  inv000aa1n06x5               g048(.a(new_n138), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n141), .b(new_n144), .c(new_n143), .d(new_n125), .o1(new_n145));
  nor042aa1n06x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand42aa1d28x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nanb02aa1n06x5               g052(.a(new_n146), .b(new_n147), .out0(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  aoib12aa1n02x5               g054(.a(new_n139), .b(new_n147), .c(new_n146), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n139), .o1(new_n151));
  inv020aa1n03x5               g056(.a(new_n141), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n151), .b(new_n152), .c(new_n135), .d(new_n138), .o1(new_n153));
  aoi022aa1n03x5               g058(.a(new_n153), .b(new_n149), .c(new_n145), .d(new_n150), .o1(\s[12] ));
  nano23aa1d12x5               g059(.a(new_n139), .b(new_n146), .c(new_n147), .d(new_n140), .out0(new_n155));
  nand23aa1d12x5               g060(.a(new_n155), .b(new_n123), .c(new_n125), .o1(new_n156));
  inv030aa1n02x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n122), .c(new_n117), .d(new_n108), .o1(new_n158));
  tech160nm_fiaoi012aa1n04x5   g063(.a(new_n146), .b(new_n139), .c(new_n147), .o1(new_n159));
  aobi12aa1n12x5               g064(.a(new_n159), .b(new_n155), .c(new_n144), .out0(new_n160));
  xnrc02aa1n12x5               g065(.a(\b[12] ), .b(\a[13] ), .out0(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n158), .c(new_n160), .out0(\s[13] ));
  nor042aa1n03x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  inv000aa1n02x5               g069(.a(new_n164), .o1(new_n165));
  oai013aa1n03x5               g070(.a(new_n159), .b(new_n152), .c(new_n138), .d(new_n148), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n162), .b(new_n166), .c(new_n134), .d(new_n157), .o1(new_n167));
  xnrc02aa1n12x5               g072(.a(\b[13] ), .b(\a[14] ), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n167), .c(new_n165), .out0(\s[14] ));
  nor042aa1n06x5               g075(.a(new_n168), .b(new_n161), .o1(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n166), .c(new_n134), .d(new_n157), .o1(new_n172));
  oaoi03aa1n02x5               g077(.a(\a[14] ), .b(\b[13] ), .c(new_n165), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  xorc02aa1n02x5               g079(.a(\a[15] ), .b(\b[14] ), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n172), .c(new_n174), .out0(\s[15] ));
  nanp02aa1n02x5               g081(.a(new_n158), .b(new_n160), .o1(new_n177));
  aoai13aa1n03x5               g082(.a(new_n175), .b(new_n173), .c(new_n177), .d(new_n171), .o1(new_n178));
  xorc02aa1n02x5               g083(.a(\a[16] ), .b(\b[15] ), .out0(new_n179));
  inv040aa1d30x5               g084(.a(\a[15] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(\b[14] ), .o1(new_n181));
  orn002aa1n06x5               g086(.a(\a[16] ), .b(\b[15] ), .o(new_n182));
  and002aa1n06x5               g087(.a(\b[15] ), .b(\a[16] ), .o(new_n183));
  aboi22aa1n03x5               g088(.a(new_n183), .b(new_n182), .c(new_n181), .d(new_n180), .out0(new_n184));
  tech160nm_finand02aa1n03p5x5 g089(.a(new_n181), .b(new_n180), .o1(new_n185));
  inv000aa1n02x5               g090(.a(new_n175), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n172), .d(new_n174), .o1(new_n187));
  aoi022aa1n02x7               g092(.a(new_n187), .b(new_n179), .c(new_n178), .d(new_n184), .o1(\s[16] ));
  nand02aa1n02x5               g093(.a(\b[14] ), .b(\a[15] ), .o1(new_n189));
  nano32aa1n03x7               g094(.a(new_n183), .b(new_n182), .c(new_n185), .d(new_n189), .out0(new_n190));
  nano22aa1d15x5               g095(.a(new_n156), .b(new_n171), .c(new_n190), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n122), .c(new_n117), .d(new_n108), .o1(new_n192));
  nand02aa1d06x5               g097(.a(new_n171), .b(new_n190), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[13] ), .b(\a[14] ), .o1(new_n194));
  oai022aa1n06x5               g099(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n195));
  nand43aa1n02x5               g100(.a(new_n195), .b(new_n194), .c(new_n189), .o1(new_n196));
  aoai13aa1n04x5               g101(.a(new_n182), .b(new_n183), .c(new_n196), .d(new_n185), .o1(new_n197));
  oab012aa1n06x5               g102(.a(new_n197), .b(new_n160), .c(new_n193), .out0(new_n198));
  nanp02aa1n09x5               g103(.a(new_n192), .b(new_n198), .o1(new_n199));
  xorc02aa1n12x5               g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  aoi113aa1n02x5               g105(.a(new_n197), .b(new_n200), .c(new_n166), .d(new_n171), .e(new_n190), .o1(new_n201));
  aoi022aa1n02x5               g106(.a(new_n199), .b(new_n200), .c(new_n192), .d(new_n201), .o1(\s[17] ));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  nanb02aa1n12x5               g108(.a(\b[16] ), .b(new_n203), .out0(new_n204));
  oabi12aa1n18x5               g109(.a(new_n197), .b(new_n160), .c(new_n193), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n200), .b(new_n205), .c(new_n134), .d(new_n191), .o1(new_n206));
  nor002aa1d32x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nand02aa1d28x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  norb02aa1n15x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n206), .c(new_n204), .out0(\s[18] ));
  and002aa1n02x5               g115(.a(new_n200), .b(new_n209), .o(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n205), .c(new_n134), .d(new_n191), .o1(new_n212));
  oaoi03aa1n02x5               g117(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n213));
  inv000aa1n02x5               g118(.a(new_n213), .o1(new_n214));
  nor042aa1d18x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nand42aa1n16x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norb02aa1d21x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n212), .c(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g124(.a(new_n217), .b(new_n213), .c(new_n199), .d(new_n211), .o1(new_n220));
  nor002aa1n12x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand02aa1d28x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n03x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[19] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[18] ), .o1(new_n225));
  aboi22aa1n03x5               g130(.a(new_n221), .b(new_n222), .c(new_n224), .d(new_n225), .out0(new_n226));
  inv040aa1n06x5               g131(.a(new_n215), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n217), .o1(new_n228));
  aoai13aa1n02x7               g133(.a(new_n227), .b(new_n228), .c(new_n212), .d(new_n214), .o1(new_n229));
  aoi022aa1n03x5               g134(.a(new_n229), .b(new_n223), .c(new_n220), .d(new_n226), .o1(\s[20] ));
  nano32aa1n03x7               g135(.a(new_n228), .b(new_n200), .c(new_n223), .d(new_n209), .out0(new_n231));
  aoai13aa1n06x5               g136(.a(new_n231), .b(new_n205), .c(new_n134), .d(new_n191), .o1(new_n232));
  nanb03aa1n12x5               g137(.a(new_n221), .b(new_n222), .c(new_n216), .out0(new_n233));
  nor042aa1n09x5               g138(.a(\b[16] ), .b(\a[17] ), .o1(new_n234));
  oai112aa1n06x5               g139(.a(new_n227), .b(new_n208), .c(new_n207), .d(new_n234), .o1(new_n235));
  aoi012aa1n09x5               g140(.a(new_n221), .b(new_n215), .c(new_n222), .o1(new_n236));
  oai012aa1n18x5               g141(.a(new_n236), .b(new_n235), .c(new_n233), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  nor002aa1d32x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  nand02aa1n08x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  norb02aa1n12x5               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  xnbna2aa1n03x5               g146(.a(new_n241), .b(new_n232), .c(new_n238), .out0(\s[21] ));
  aoai13aa1n03x5               g147(.a(new_n241), .b(new_n237), .c(new_n199), .d(new_n231), .o1(new_n243));
  nor042aa1n03x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nand02aa1n08x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  aoib12aa1n02x5               g151(.a(new_n239), .b(new_n245), .c(new_n244), .out0(new_n247));
  inv040aa1n08x5               g152(.a(new_n239), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n241), .o1(new_n249));
  aoai13aa1n02x7               g154(.a(new_n248), .b(new_n249), .c(new_n232), .d(new_n238), .o1(new_n250));
  aoi022aa1n03x5               g155(.a(new_n250), .b(new_n246), .c(new_n243), .d(new_n247), .o1(\s[22] ));
  inv000aa1n02x5               g156(.a(new_n231), .o1(new_n252));
  nano22aa1n03x7               g157(.a(new_n252), .b(new_n241), .c(new_n246), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n205), .c(new_n134), .d(new_n191), .o1(new_n254));
  nano23aa1d12x5               g159(.a(new_n239), .b(new_n244), .c(new_n245), .d(new_n240), .out0(new_n255));
  oaoi03aa1n12x5               g160(.a(\a[22] ), .b(\b[21] ), .c(new_n248), .o1(new_n256));
  aoi012aa1n06x5               g161(.a(new_n256), .b(new_n237), .c(new_n255), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  xnbna2aa1n03x5               g163(.a(new_n258), .b(new_n254), .c(new_n257), .out0(\s[23] ));
  inv000aa1n02x5               g164(.a(new_n257), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n258), .b(new_n260), .c(new_n199), .d(new_n253), .o1(new_n261));
  tech160nm_fixorc02aa1n02p5x5 g166(.a(\a[24] ), .b(\b[23] ), .out0(new_n262));
  nor042aa1n09x5               g167(.a(\b[22] ), .b(\a[23] ), .o1(new_n263));
  norp02aa1n02x5               g168(.a(new_n262), .b(new_n263), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n263), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n258), .o1(new_n266));
  aoai13aa1n02x7               g171(.a(new_n265), .b(new_n266), .c(new_n254), .d(new_n257), .o1(new_n267));
  aoi022aa1n03x5               g172(.a(new_n267), .b(new_n262), .c(new_n261), .d(new_n264), .o1(\s[24] ));
  and002aa1n12x5               g173(.a(new_n262), .b(new_n258), .o(new_n269));
  nano22aa1n02x5               g174(.a(new_n252), .b(new_n269), .c(new_n255), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n205), .c(new_n134), .d(new_n191), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n269), .b(new_n256), .c(new_n237), .d(new_n255), .o1(new_n272));
  nano22aa1n02x4               g177(.a(new_n221), .b(new_n216), .c(new_n222), .out0(new_n273));
  oai012aa1n02x5               g178(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .o1(new_n274));
  oab012aa1n04x5               g179(.a(new_n274), .b(new_n234), .c(new_n207), .out0(new_n275));
  inv000aa1n02x5               g180(.a(new_n236), .o1(new_n276));
  aoai13aa1n04x5               g181(.a(new_n255), .b(new_n276), .c(new_n275), .d(new_n273), .o1(new_n277));
  inv000aa1n02x5               g182(.a(new_n256), .o1(new_n278));
  inv000aa1n06x5               g183(.a(new_n269), .o1(new_n279));
  oao003aa1n02x5               g184(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .carry(new_n280));
  aoai13aa1n12x5               g185(.a(new_n280), .b(new_n279), .c(new_n277), .d(new_n278), .o1(new_n281));
  inv030aa1n02x5               g186(.a(new_n281), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  tech160nm_fiaoi012aa1n05x5   g189(.a(new_n284), .b(new_n271), .c(new_n282), .o1(new_n285));
  norb02aa1n02x5               g190(.a(new_n280), .b(new_n283), .out0(new_n286));
  aoi013aa1n02x4               g191(.a(new_n285), .b(new_n272), .c(new_n271), .d(new_n286), .o1(\s[25] ));
  aoai13aa1n03x5               g192(.a(new_n283), .b(new_n281), .c(new_n199), .d(new_n270), .o1(new_n288));
  tech160nm_fixorc02aa1n02p5x5 g193(.a(\a[26] ), .b(\b[25] ), .out0(new_n289));
  nor042aa1n09x5               g194(.a(\b[24] ), .b(\a[25] ), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n289), .b(new_n290), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n290), .o1(new_n292));
  aoai13aa1n04x5               g197(.a(new_n292), .b(new_n284), .c(new_n271), .d(new_n282), .o1(new_n293));
  aoi022aa1n02x7               g198(.a(new_n293), .b(new_n289), .c(new_n288), .d(new_n291), .o1(\s[26] ));
  and002aa1n12x5               g199(.a(new_n289), .b(new_n283), .o(new_n295));
  nano32aa1d12x5               g200(.a(new_n252), .b(new_n295), .c(new_n255), .d(new_n269), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n205), .c(new_n134), .d(new_n191), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n295), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[26] ), .b(\b[25] ), .c(new_n292), .carry(new_n299));
  aoai13aa1n04x5               g204(.a(new_n299), .b(new_n298), .c(new_n272), .d(new_n280), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  aoai13aa1n06x5               g206(.a(new_n301), .b(new_n300), .c(new_n199), .d(new_n296), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n299), .o1(new_n303));
  aoi112aa1n02x5               g208(.a(new_n301), .b(new_n303), .c(new_n281), .d(new_n295), .o1(new_n304));
  aobi12aa1n02x7               g209(.a(new_n302), .b(new_n304), .c(new_n297), .out0(\s[27] ));
  xorc02aa1n12x5               g210(.a(\a[28] ), .b(\b[27] ), .out0(new_n306));
  norp02aa1n02x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  norp02aa1n02x5               g212(.a(new_n306), .b(new_n307), .o1(new_n308));
  aoi012aa1n12x5               g213(.a(new_n303), .b(new_n281), .c(new_n295), .o1(new_n309));
  inv000aa1n03x5               g214(.a(new_n307), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n301), .o1(new_n311));
  aoai13aa1n02x7               g216(.a(new_n310), .b(new_n311), .c(new_n309), .d(new_n297), .o1(new_n312));
  aoi022aa1n03x5               g217(.a(new_n312), .b(new_n306), .c(new_n302), .d(new_n308), .o1(\s[28] ));
  and002aa1n02x5               g218(.a(new_n306), .b(new_n301), .o(new_n314));
  aoai13aa1n02x5               g219(.a(new_n314), .b(new_n300), .c(new_n199), .d(new_n296), .o1(new_n315));
  inv000aa1n02x5               g220(.a(new_n314), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n310), .carry(new_n317));
  aoai13aa1n02x7               g222(.a(new_n317), .b(new_n316), .c(new_n309), .d(new_n297), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .out0(new_n319));
  norb02aa1n02x5               g224(.a(new_n317), .b(new_n319), .out0(new_n320));
  aoi022aa1n03x5               g225(.a(new_n318), .b(new_n319), .c(new_n315), .d(new_n320), .o1(\s[29] ));
  xnrb03aa1n02x5               g226(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g227(.a(new_n311), .b(new_n306), .c(new_n319), .out0(new_n323));
  aoai13aa1n02x5               g228(.a(new_n323), .b(new_n300), .c(new_n199), .d(new_n296), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n323), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .carry(new_n326));
  aoai13aa1n02x7               g231(.a(new_n326), .b(new_n325), .c(new_n309), .d(new_n297), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .out0(new_n328));
  norb02aa1n02x5               g233(.a(new_n326), .b(new_n328), .out0(new_n329));
  aoi022aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n324), .d(new_n329), .o1(\s[30] ));
  nano32aa1d15x5               g235(.a(new_n311), .b(new_n328), .c(new_n306), .d(new_n319), .out0(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n300), .c(new_n199), .d(new_n296), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  and002aa1n02x5               g238(.a(\b[29] ), .b(\a[30] ), .o(new_n334));
  oabi12aa1n02x5               g239(.a(new_n333), .b(\a[30] ), .c(\b[29] ), .out0(new_n335));
  oab012aa1n02x4               g240(.a(new_n335), .b(new_n326), .c(new_n334), .out0(new_n336));
  inv000aa1d42x5               g241(.a(new_n331), .o1(new_n337));
  oao003aa1n02x5               g242(.a(\a[30] ), .b(\b[29] ), .c(new_n326), .carry(new_n338));
  aoai13aa1n02x7               g243(.a(new_n338), .b(new_n337), .c(new_n309), .d(new_n297), .o1(new_n339));
  aoi022aa1n03x5               g244(.a(new_n339), .b(new_n333), .c(new_n332), .d(new_n336), .o1(\s[31] ));
  orn002aa1n02x5               g245(.a(\a[2] ), .b(\b[1] ), .o(new_n341));
  nanp02aa1n02x5               g246(.a(\b[1] ), .b(\a[2] ), .o1(new_n342));
  nanb03aa1n02x5               g247(.a(new_n99), .b(new_n341), .c(new_n342), .out0(new_n343));
  norb02aa1n02x5               g248(.a(new_n104), .b(new_n103), .out0(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n344), .b(new_n343), .c(new_n341), .out0(\s[3] ));
  obai22aa1n02x7               g250(.a(new_n101), .b(new_n102), .c(\a[3] ), .d(\b[2] ), .out0(new_n346));
  aoi012aa1n02x5               g251(.a(new_n346), .b(new_n100), .c(new_n344), .o1(new_n347));
  oaoi13aa1n02x5               g252(.a(new_n347), .b(new_n108), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g253(.a(new_n114), .b(new_n106), .c(new_n107), .out0(\s[5] ));
  orn002aa1n02x5               g254(.a(\a[5] ), .b(\b[4] ), .o(new_n350));
  nanp02aa1n03x5               g255(.a(new_n108), .b(new_n114), .o1(new_n351));
  nona22aa1n03x5               g256(.a(new_n351), .b(new_n120), .c(new_n118), .out0(new_n352));
  aoai13aa1n02x5               g257(.a(new_n352), .b(new_n115), .c(new_n350), .d(new_n351), .o1(\s[6] ));
  aoi022aa1n02x5               g258(.a(new_n352), .b(new_n119), .c(new_n128), .d(new_n112), .o1(new_n354));
  nona23aa1n06x5               g259(.a(new_n352), .b(new_n112), .c(new_n111), .d(new_n118), .out0(new_n355));
  norb02aa1n02x5               g260(.a(new_n355), .b(new_n354), .out0(\s[7] ));
  norb02aa1n02x5               g261(.a(new_n109), .b(new_n110), .out0(new_n357));
  xnbna2aa1n03x5               g262(.a(new_n357), .b(new_n355), .c(new_n128), .out0(\s[8] ));
  xorb03aa1n02x5               g263(.a(new_n134), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


