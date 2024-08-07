// Benchmark "adder" written by ABC on Thu Jul 18 03:03:21 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
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
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n341, new_n342, new_n343, new_n345,
    new_n346, new_n348, new_n350, new_n351, new_n353, new_n354, new_n357;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n03x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  inv000aa1d42x5               g002(.a(\a[3] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[2] ), .o1(new_n99));
  tech160nm_finand02aa1n03p5x5 g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nor002aa1n03x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  aoai13aa1n04x5               g006(.a(new_n100), .b(new_n101), .c(new_n99), .d(new_n98), .o1(new_n102));
  norb02aa1n03x5               g007(.a(new_n100), .b(new_n101), .out0(new_n103));
  and002aa1n12x5               g008(.a(\b[0] ), .b(\a[1] ), .o(new_n104));
  tech160nm_fioaoi03aa1n03p5x5 g009(.a(\a[2] ), .b(\b[1] ), .c(new_n104), .o1(new_n105));
  xorc02aa1n06x5               g010(.a(\a[3] ), .b(\b[2] ), .out0(new_n106));
  nanp03aa1n06x5               g011(.a(new_n105), .b(new_n106), .c(new_n103), .o1(new_n107));
  nand02aa1n04x5               g012(.a(new_n107), .b(new_n102), .o1(new_n108));
  nor002aa1n03x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nanp02aa1n06x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor042aa1n06x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nano23aa1n06x5               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  nor042aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand42aa1n06x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nand02aa1n06x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nor002aa1n20x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  norb02aa1n06x4               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  nand23aa1d12x5               g024(.a(new_n113), .b(new_n116), .c(new_n119), .o1(new_n120));
  inv000aa1d42x5               g025(.a(new_n120), .o1(new_n121));
  norb03aa1n03x5               g026(.a(new_n115), .b(new_n114), .c(new_n118), .out0(new_n122));
  inv000aa1n03x5               g027(.a(new_n111), .o1(new_n123));
  oaoi03aa1n09x5               g028(.a(\a[6] ), .b(\b[5] ), .c(new_n123), .o1(new_n124));
  nanp03aa1n02x5               g029(.a(new_n124), .b(new_n117), .c(new_n122), .o1(new_n125));
  oaib12aa1n02x5               g030(.a(new_n125), .b(new_n122), .c(new_n115), .out0(new_n126));
  nand42aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nand02aa1n02x5               g032(.a(new_n97), .b(new_n127), .o1(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n126), .c(new_n121), .d(new_n108), .o1(new_n130));
  norp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand42aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g039(.a(new_n118), .o1(new_n135));
  oaoi03aa1n02x5               g040(.a(\a[8] ), .b(\b[7] ), .c(new_n135), .o1(new_n136));
  aoi013aa1n06x4               g041(.a(new_n136), .b(new_n124), .c(new_n122), .d(new_n117), .o1(new_n137));
  aoai13aa1n12x5               g042(.a(new_n137), .b(new_n120), .c(new_n107), .d(new_n102), .o1(new_n138));
  tech160nm_fioaoi03aa1n03p5x5 g043(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n139));
  nano32aa1n03x7               g044(.a(new_n131), .b(new_n97), .c(new_n132), .d(new_n127), .out0(new_n140));
  tech160nm_fixorc02aa1n02p5x5 g045(.a(\a[11] ), .b(\b[10] ), .out0(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n139), .c(new_n138), .d(new_n140), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(new_n139), .b(new_n141), .c(new_n138), .d(new_n140), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(\s[11] ));
  nor042aa1n04x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  tech160nm_fixorc02aa1n03p5x5 g051(.a(\a[12] ), .b(\b[11] ), .out0(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n147), .b(new_n142), .c(new_n146), .out0(\s[12] ));
  nano32aa1n03x5               g053(.a(new_n128), .b(new_n147), .c(new_n133), .d(new_n141), .out0(new_n149));
  nanp02aa1n02x5               g054(.a(\b[10] ), .b(\a[11] ), .o1(new_n150));
  norp02aa1n02x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanb03aa1n02x5               g057(.a(new_n151), .b(new_n152), .c(new_n150), .out0(new_n153));
  nanb03aa1n06x5               g058(.a(new_n153), .b(new_n139), .c(new_n146), .out0(new_n154));
  tech160nm_fiaoi012aa1n04x5   g059(.a(new_n151), .b(new_n145), .c(new_n152), .o1(new_n155));
  nanp02aa1n03x5               g060(.a(new_n154), .b(new_n155), .o1(new_n156));
  nor042aa1n09x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand42aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanb02aa1n02x5               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n156), .c(new_n138), .d(new_n149), .o1(new_n161));
  aoi112aa1n02x5               g066(.a(new_n160), .b(new_n156), .c(new_n138), .d(new_n149), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(\s[13] ));
  inv000aa1d42x5               g068(.a(new_n157), .o1(new_n164));
  nor042aa1n04x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nanp02aa1n04x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n161), .c(new_n164), .out0(\s[14] ));
  nano23aa1n06x5               g073(.a(new_n157), .b(new_n165), .c(new_n166), .d(new_n158), .out0(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n156), .c(new_n138), .d(new_n149), .o1(new_n170));
  oai012aa1n02x5               g075(.a(new_n166), .b(new_n165), .c(new_n157), .o1(new_n171));
  xorc02aa1n12x5               g076(.a(\a[15] ), .b(\b[14] ), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n170), .c(new_n171), .out0(\s[15] ));
  nand42aa1n02x5               g078(.a(new_n170), .b(new_n171), .o1(new_n174));
  nand02aa1n02x5               g079(.a(new_n174), .b(new_n172), .o1(new_n175));
  xorc02aa1n12x5               g080(.a(\a[16] ), .b(\b[15] ), .out0(new_n176));
  inv000aa1d42x5               g081(.a(\a[15] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\b[14] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\a[16] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[15] ), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n179), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  aoi022aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .d(new_n177), .o1(new_n183));
  nor042aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  inv000aa1n02x5               g089(.a(new_n184), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n172), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n170), .d(new_n171), .o1(new_n187));
  aoi022aa1n03x5               g092(.a(new_n187), .b(new_n176), .c(new_n175), .d(new_n183), .o1(\s[16] ));
  nand23aa1d12x5               g093(.a(new_n169), .b(new_n172), .c(new_n176), .o1(new_n189));
  nano32aa1d12x5               g094(.a(new_n189), .b(new_n140), .c(new_n141), .d(new_n147), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n126), .c(new_n121), .d(new_n108), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n189), .o1(new_n192));
  oai112aa1n02x5               g097(.a(new_n181), .b(new_n182), .c(new_n178), .d(new_n177), .o1(new_n193));
  oai112aa1n06x5               g098(.a(new_n185), .b(new_n166), .c(new_n165), .d(new_n157), .o1(new_n194));
  oaoi03aa1n02x5               g099(.a(new_n179), .b(new_n180), .c(new_n184), .o1(new_n195));
  oa0012aa1n12x5               g100(.a(new_n195), .b(new_n194), .c(new_n193), .o(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  tech160nm_fiaoi012aa1n05x5   g102(.a(new_n197), .b(new_n156), .c(new_n192), .o1(new_n198));
  nanp02aa1n12x5               g103(.a(new_n191), .b(new_n198), .o1(new_n199));
  xorc02aa1n12x5               g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  aoi112aa1n02x5               g105(.a(new_n197), .b(new_n200), .c(new_n156), .d(new_n192), .o1(new_n201));
  aoi022aa1n02x5               g106(.a(new_n199), .b(new_n200), .c(new_n191), .d(new_n201), .o1(\s[17] ));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(\b[16] ), .b(new_n203), .out0(new_n204));
  aoai13aa1n06x5               g109(.a(new_n196), .b(new_n189), .c(new_n154), .d(new_n155), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n200), .b(new_n205), .c(new_n138), .d(new_n190), .o1(new_n206));
  nor042aa1n03x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nand02aa1d06x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  norb02aa1n06x4               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n206), .c(new_n204), .out0(\s[18] ));
  and002aa1n02x5               g115(.a(new_n200), .b(new_n209), .o(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n205), .c(new_n138), .d(new_n190), .o1(new_n212));
  oaoi03aa1n02x5               g117(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  nor042aa1d18x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nand02aa1n04x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norb02aa1n12x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n212), .c(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g124(.a(new_n217), .b(new_n213), .c(new_n199), .d(new_n211), .o1(new_n220));
  nor002aa1n06x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand02aa1d06x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n03x4               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[19] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[18] ), .o1(new_n225));
  aboi22aa1n03x5               g130(.a(new_n221), .b(new_n222), .c(new_n224), .d(new_n225), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n215), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n217), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n227), .b(new_n228), .c(new_n212), .d(new_n214), .o1(new_n229));
  aoi022aa1n03x5               g134(.a(new_n229), .b(new_n223), .c(new_n220), .d(new_n226), .o1(\s[20] ));
  nano32aa1n03x7               g135(.a(new_n228), .b(new_n200), .c(new_n223), .d(new_n209), .out0(new_n231));
  aoai13aa1n06x5               g136(.a(new_n231), .b(new_n205), .c(new_n138), .d(new_n190), .o1(new_n232));
  nanb03aa1d18x5               g137(.a(new_n221), .b(new_n222), .c(new_n216), .out0(new_n233));
  nor042aa1n02x5               g138(.a(\b[16] ), .b(\a[17] ), .o1(new_n234));
  oai112aa1n06x5               g139(.a(new_n227), .b(new_n208), .c(new_n207), .d(new_n234), .o1(new_n235));
  aoi012aa1n12x5               g140(.a(new_n221), .b(new_n215), .c(new_n222), .o1(new_n236));
  oai012aa1d24x5               g141(.a(new_n236), .b(new_n235), .c(new_n233), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  nor002aa1n16x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  nanp02aa1n04x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  norb02aa1d27x5               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  xnbna2aa1n03x5               g146(.a(new_n241), .b(new_n232), .c(new_n238), .out0(\s[21] ));
  aoai13aa1n06x5               g147(.a(new_n241), .b(new_n237), .c(new_n199), .d(new_n231), .o1(new_n243));
  nor042aa1n02x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nanp02aa1n04x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  aoib12aa1n02x5               g151(.a(new_n239), .b(new_n245), .c(new_n244), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n239), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n241), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n248), .b(new_n249), .c(new_n232), .d(new_n238), .o1(new_n250));
  aoi022aa1n02x5               g155(.a(new_n250), .b(new_n246), .c(new_n243), .d(new_n247), .o1(\s[22] ));
  inv020aa1n02x5               g156(.a(new_n231), .o1(new_n252));
  nano22aa1n03x7               g157(.a(new_n252), .b(new_n241), .c(new_n246), .out0(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n205), .c(new_n138), .d(new_n190), .o1(new_n254));
  nano22aa1n03x7               g159(.a(new_n221), .b(new_n216), .c(new_n222), .out0(new_n255));
  oai012aa1n02x5               g160(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .o1(new_n256));
  oab012aa1n03x5               g161(.a(new_n256), .b(new_n234), .c(new_n207), .out0(new_n257));
  inv020aa1n03x5               g162(.a(new_n236), .o1(new_n258));
  nano23aa1n09x5               g163(.a(new_n239), .b(new_n244), .c(new_n245), .d(new_n240), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n258), .c(new_n257), .d(new_n255), .o1(new_n260));
  aoi012aa1n12x5               g165(.a(new_n244), .b(new_n239), .c(new_n245), .o1(new_n261));
  nand42aa1n03x5               g166(.a(new_n260), .b(new_n261), .o1(new_n262));
  xorc02aa1n12x5               g167(.a(\a[23] ), .b(\b[22] ), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n262), .c(new_n199), .d(new_n253), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n261), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n263), .b(new_n265), .c(new_n237), .d(new_n259), .o1(new_n266));
  aobi12aa1n02x7               g171(.a(new_n264), .b(new_n266), .c(new_n254), .out0(\s[23] ));
  xorc02aa1n02x5               g172(.a(\a[24] ), .b(\b[23] ), .out0(new_n268));
  nor042aa1n06x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n268), .b(new_n269), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n262), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n269), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n263), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n272), .b(new_n273), .c(new_n254), .d(new_n271), .o1(new_n274));
  aoi022aa1n03x5               g179(.a(new_n274), .b(new_n268), .c(new_n264), .d(new_n270), .o1(\s[24] ));
  and002aa1n06x5               g180(.a(new_n268), .b(new_n263), .o(new_n276));
  nano22aa1n02x5               g181(.a(new_n252), .b(new_n276), .c(new_n259), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n205), .c(new_n138), .d(new_n190), .o1(new_n278));
  inv030aa1n02x5               g183(.a(new_n276), .o1(new_n279));
  oao003aa1n02x5               g184(.a(\a[24] ), .b(\b[23] ), .c(new_n272), .carry(new_n280));
  aoai13aa1n12x5               g185(.a(new_n280), .b(new_n279), .c(new_n260), .d(new_n261), .o1(new_n281));
  xorc02aa1n12x5               g186(.a(\a[25] ), .b(\b[24] ), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n281), .c(new_n199), .d(new_n277), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n276), .b(new_n265), .c(new_n237), .d(new_n259), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n282), .o1(new_n285));
  and003aa1n02x5               g190(.a(new_n284), .b(new_n285), .c(new_n280), .o(new_n286));
  aobi12aa1n03x7               g191(.a(new_n283), .b(new_n286), .c(new_n278), .out0(\s[25] ));
  tech160nm_fixorc02aa1n03p5x5 g192(.a(\a[26] ), .b(\b[25] ), .out0(new_n288));
  nor042aa1n03x5               g193(.a(\b[24] ), .b(\a[25] ), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n288), .b(new_n289), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n281), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n289), .o1(new_n292));
  aoai13aa1n02x7               g197(.a(new_n292), .b(new_n285), .c(new_n278), .d(new_n291), .o1(new_n293));
  aoi022aa1n03x5               g198(.a(new_n293), .b(new_n288), .c(new_n283), .d(new_n290), .o1(\s[26] ));
  and002aa1n18x5               g199(.a(new_n288), .b(new_n282), .o(new_n295));
  nano32aa1n03x7               g200(.a(new_n252), .b(new_n295), .c(new_n259), .d(new_n276), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n205), .c(new_n138), .d(new_n190), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n295), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[26] ), .b(\b[25] ), .c(new_n292), .carry(new_n299));
  aoai13aa1n04x5               g204(.a(new_n299), .b(new_n298), .c(new_n284), .d(new_n280), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  aoai13aa1n06x5               g206(.a(new_n301), .b(new_n300), .c(new_n199), .d(new_n296), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n299), .o1(new_n303));
  aoi112aa1n02x5               g208(.a(new_n301), .b(new_n303), .c(new_n281), .d(new_n295), .o1(new_n304));
  aobi12aa1n03x7               g209(.a(new_n302), .b(new_n304), .c(new_n297), .out0(\s[27] ));
  tech160nm_fixorc02aa1n02p5x5 g210(.a(\a[28] ), .b(\b[27] ), .out0(new_n306));
  norp02aa1n02x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  norp02aa1n02x5               g212(.a(new_n306), .b(new_n307), .o1(new_n308));
  aoi012aa1n09x5               g213(.a(new_n303), .b(new_n281), .c(new_n295), .o1(new_n309));
  inv000aa1n03x5               g214(.a(new_n307), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n301), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n310), .b(new_n311), .c(new_n309), .d(new_n297), .o1(new_n312));
  aoi022aa1n03x5               g217(.a(new_n312), .b(new_n306), .c(new_n302), .d(new_n308), .o1(\s[28] ));
  and002aa1n02x5               g218(.a(new_n306), .b(new_n301), .o(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n300), .c(new_n199), .d(new_n296), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  oao003aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n310), .carry(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n314), .o1(new_n319));
  aoai13aa1n03x5               g224(.a(new_n317), .b(new_n319), .c(new_n309), .d(new_n297), .o1(new_n320));
  aoi022aa1n03x5               g225(.a(new_n320), .b(new_n316), .c(new_n315), .d(new_n318), .o1(\s[29] ));
  xnrb03aa1n02x5               g226(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g227(.a(new_n311), .b(new_n306), .c(new_n316), .out0(new_n323));
  aoai13aa1n02x5               g228(.a(new_n323), .b(new_n300), .c(new_n199), .d(new_n296), .o1(new_n324));
  inv000aa1n02x5               g229(.a(new_n323), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .carry(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n325), .c(new_n309), .d(new_n297), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .out0(new_n328));
  norb02aa1n02x5               g233(.a(new_n326), .b(new_n328), .out0(new_n329));
  aoi022aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n324), .d(new_n329), .o1(\s[30] ));
  nano32aa1n03x7               g235(.a(new_n311), .b(new_n328), .c(new_n306), .d(new_n316), .out0(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n300), .c(new_n199), .d(new_n296), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  and002aa1n02x5               g238(.a(\b[29] ), .b(\a[30] ), .o(new_n334));
  oabi12aa1n02x5               g239(.a(new_n333), .b(\a[30] ), .c(\b[29] ), .out0(new_n335));
  oab012aa1n02x4               g240(.a(new_n335), .b(new_n326), .c(new_n334), .out0(new_n336));
  inv000aa1d42x5               g241(.a(new_n331), .o1(new_n337));
  oao003aa1n02x5               g242(.a(\a[30] ), .b(\b[29] ), .c(new_n326), .carry(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n337), .c(new_n309), .d(new_n297), .o1(new_n339));
  aoi022aa1n03x5               g244(.a(new_n339), .b(new_n333), .c(new_n332), .d(new_n336), .o1(\s[31] ));
  orn002aa1n02x5               g245(.a(\a[2] ), .b(\b[1] ), .o(new_n341));
  nanp02aa1n02x5               g246(.a(\b[1] ), .b(\a[2] ), .o1(new_n342));
  nanb03aa1n02x5               g247(.a(new_n104), .b(new_n341), .c(new_n342), .out0(new_n343));
  xnbna2aa1n03x5               g248(.a(new_n106), .b(new_n343), .c(new_n341), .out0(\s[3] ));
  obai22aa1n02x7               g249(.a(new_n100), .b(new_n101), .c(\a[3] ), .d(\b[2] ), .out0(new_n345));
  aoi012aa1n02x5               g250(.a(new_n345), .b(new_n105), .c(new_n106), .o1(new_n346));
  oaoi13aa1n02x5               g251(.a(new_n346), .b(new_n108), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  norb02aa1n02x5               g252(.a(new_n112), .b(new_n111), .out0(new_n348));
  xnbna2aa1n03x5               g253(.a(new_n348), .b(new_n107), .c(new_n102), .out0(\s[5] ));
  norb02aa1n02x5               g254(.a(new_n110), .b(new_n109), .out0(new_n350));
  nanp02aa1n02x5               g255(.a(new_n108), .b(new_n348), .o1(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n350), .b(new_n351), .c(new_n123), .out0(\s[6] ));
  aoai13aa1n02x5               g257(.a(new_n119), .b(new_n124), .c(new_n108), .d(new_n113), .o1(new_n353));
  aoi112aa1n02x5               g258(.a(new_n124), .b(new_n119), .c(new_n108), .d(new_n113), .o1(new_n354));
  norb02aa1n02x5               g259(.a(new_n353), .b(new_n354), .out0(\s[7] ));
  xnbna2aa1n03x5               g260(.a(new_n116), .b(new_n353), .c(new_n135), .out0(\s[8] ));
  aoi112aa1n02x5               g261(.a(new_n126), .b(new_n129), .c(new_n121), .d(new_n108), .o1(new_n357));
  aoi012aa1n02x5               g262(.a(new_n357), .b(new_n138), .c(new_n129), .o1(\s[9] ));
endmodule


