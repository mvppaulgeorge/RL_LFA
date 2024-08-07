// Benchmark "adder" written by ABC on Thu Jul 18 15:17:59 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n179, new_n180, new_n181,
    new_n182, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n311, new_n312,
    new_n314, new_n315, new_n316, new_n317, new_n318, new_n320;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n06x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nor002aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oai012aa1n12x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  xnrc02aa1n12x5               g006(.a(\b[3] ), .b(\a[4] ), .out0(new_n102));
  nor002aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanb02aa1n06x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  inv000aa1d42x5               g010(.a(\a[3] ), .o1(new_n106));
  nanb02aa1n02x5               g011(.a(\b[2] ), .b(new_n106), .out0(new_n107));
  oao003aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .carry(new_n108));
  oai013aa1d12x5               g013(.a(new_n108), .b(new_n102), .c(new_n101), .d(new_n105), .o1(new_n109));
  xorc02aa1n02x5               g014(.a(\a[6] ), .b(\b[5] ), .out0(new_n110));
  tech160nm_finand02aa1n03p5x5 g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor022aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor022aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n111), .b(new_n114), .c(new_n113), .d(new_n112), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norb03aa1n09x5               g021(.a(new_n110), .b(new_n115), .c(new_n116), .out0(new_n117));
  nand22aa1n03x5               g022(.a(new_n117), .b(new_n109), .o1(new_n118));
  oai022aa1n02x5               g023(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[5] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[4] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n120), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n122), .o1(new_n123));
  aboi22aa1n03x5               g028(.a(new_n115), .b(new_n123), .c(new_n111), .d(new_n119), .out0(new_n124));
  nand02aa1d08x5               g029(.a(new_n118), .b(new_n124), .o1(new_n125));
  tech160nm_finand02aa1n05x5   g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n97), .b(new_n125), .c(new_n126), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n20x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  oai012aa1d24x5               g035(.a(new_n130), .b(new_n129), .c(new_n97), .o1(new_n131));
  inv030aa1n02x5               g036(.a(new_n131), .o1(new_n132));
  nano23aa1n09x5               g037(.a(new_n97), .b(new_n129), .c(new_n130), .d(new_n126), .out0(new_n133));
  tech160nm_fiaoi012aa1n05x5   g038(.a(new_n132), .b(new_n125), .c(new_n133), .o1(new_n134));
  xnrb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n03x5               g040(.a(\a[11] ), .b(\b[10] ), .c(new_n134), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  oai012aa1n02x5               g042(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n138));
  oaib12aa1n02x5               g043(.a(new_n138), .b(new_n115), .c(new_n123), .out0(new_n139));
  norp02aa1n12x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nand02aa1n06x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nor022aa1n12x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand22aa1n12x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nano23aa1n09x5               g048(.a(new_n140), .b(new_n142), .c(new_n143), .d(new_n141), .out0(new_n144));
  nand02aa1d08x5               g049(.a(new_n144), .b(new_n133), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n139), .c(new_n117), .d(new_n109), .o1(new_n147));
  nona23aa1n09x5               g052(.a(new_n143), .b(new_n141), .c(new_n140), .d(new_n142), .out0(new_n148));
  ao0012aa1n12x5               g053(.a(new_n142), .b(new_n140), .c(new_n143), .o(new_n149));
  oabi12aa1n18x5               g054(.a(new_n149), .b(new_n148), .c(new_n131), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  xorc02aa1n12x5               g056(.a(\a[13] ), .b(\b[12] ), .out0(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n147), .c(new_n151), .out0(\s[13] ));
  orn002aa1n24x5               g058(.a(\a[13] ), .b(\b[12] ), .o(new_n154));
  xnrc02aa1n03x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  aoai13aa1n02x5               g060(.a(new_n154), .b(new_n155), .c(new_n147), .d(new_n151), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n06x5               g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  nanb02aa1n09x5               g063(.a(new_n158), .b(new_n152), .out0(new_n159));
  oaoi03aa1n12x5               g064(.a(\a[14] ), .b(\b[13] ), .c(new_n154), .o1(new_n160));
  inv000aa1n02x5               g065(.a(new_n160), .o1(new_n161));
  aoai13aa1n04x5               g066(.a(new_n161), .b(new_n159), .c(new_n147), .d(new_n151), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  xorc02aa1n12x5               g069(.a(\a[15] ), .b(\b[14] ), .out0(new_n165));
  xorc02aa1n12x5               g070(.a(\a[16] ), .b(\b[15] ), .out0(new_n166));
  aoi112aa1n02x5               g071(.a(new_n166), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n167));
  aoai13aa1n03x5               g072(.a(new_n166), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n168));
  norb02aa1n02x7               g073(.a(new_n168), .b(new_n167), .out0(\s[16] ));
  nand02aa1d06x5               g074(.a(new_n166), .b(new_n165), .o1(new_n170));
  nor043aa1d12x5               g075(.a(new_n145), .b(new_n159), .c(new_n170), .o1(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n139), .c(new_n109), .d(new_n117), .o1(new_n172));
  nor002aa1n03x5               g077(.a(new_n158), .b(new_n155), .o1(new_n173));
  aoi012aa1n12x5               g078(.a(new_n160), .b(new_n150), .c(new_n173), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n175));
  oab012aa1n02x4               g080(.a(new_n175), .b(\a[16] ), .c(\b[15] ), .out0(new_n176));
  oai112aa1n06x5               g081(.a(new_n172), .b(new_n176), .c(new_n174), .d(new_n170), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g083(.a(\a[18] ), .o1(new_n179));
  inv040aa1d32x5               g084(.a(\a[17] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(\b[16] ), .o1(new_n181));
  oaoi03aa1n03x5               g086(.a(new_n180), .b(new_n181), .c(new_n177), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[17] ), .c(new_n179), .out0(\s[18] ));
  aoai13aa1n03x5               g088(.a(new_n173), .b(new_n149), .c(new_n144), .d(new_n132), .o1(new_n184));
  aoai13aa1n06x5               g089(.a(new_n176), .b(new_n170), .c(new_n184), .d(new_n161), .o1(new_n185));
  xroi22aa1d06x4               g090(.a(new_n180), .b(\b[16] ), .c(new_n179), .d(\b[17] ), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n185), .c(new_n125), .d(new_n171), .o1(new_n187));
  oai022aa1n12x5               g092(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n188));
  oaib12aa1n06x5               g093(.a(new_n188), .b(new_n179), .c(\b[17] ), .out0(new_n189));
  nor002aa1d32x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nand42aa1d28x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  nanb02aa1n02x5               g096(.a(new_n190), .b(new_n191), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n187), .c(new_n189), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g100(.a(new_n190), .o1(new_n196));
  aoi012aa1n02x5               g101(.a(new_n192), .b(new_n187), .c(new_n189), .o1(new_n197));
  nor002aa1n10x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nand42aa1d28x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  nano22aa1n02x4               g105(.a(new_n197), .b(new_n196), .c(new_n200), .out0(new_n201));
  nand42aa1n02x5               g106(.a(new_n181), .b(new_n180), .o1(new_n202));
  oaoi03aa1n12x5               g107(.a(\a[18] ), .b(\b[17] ), .c(new_n202), .o1(new_n203));
  aoai13aa1n03x5               g108(.a(new_n193), .b(new_n203), .c(new_n177), .d(new_n186), .o1(new_n204));
  aoi012aa1n03x5               g109(.a(new_n200), .b(new_n204), .c(new_n196), .o1(new_n205));
  nor002aa1n02x5               g110(.a(new_n205), .b(new_n201), .o1(\s[20] ));
  nano23aa1n09x5               g111(.a(new_n190), .b(new_n198), .c(new_n199), .d(new_n191), .out0(new_n207));
  nand02aa1d04x5               g112(.a(new_n186), .b(new_n207), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n209), .b(new_n185), .c(new_n125), .d(new_n171), .o1(new_n210));
  nona23aa1d18x5               g115(.a(new_n199), .b(new_n191), .c(new_n190), .d(new_n198), .out0(new_n211));
  aoi012aa1n12x5               g116(.a(new_n198), .b(new_n190), .c(new_n199), .o1(new_n212));
  oai012aa1n18x5               g117(.a(new_n212), .b(new_n211), .c(new_n189), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  nor042aa1n06x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n210), .c(new_n214), .out0(\s[21] ));
  inv040aa1n02x5               g123(.a(new_n215), .o1(new_n219));
  aobi12aa1n06x5               g124(.a(new_n217), .b(new_n210), .c(new_n214), .out0(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[21] ), .b(\a[22] ), .out0(new_n221));
  nano22aa1n02x4               g126(.a(new_n220), .b(new_n219), .c(new_n221), .out0(new_n222));
  aoai13aa1n03x5               g127(.a(new_n217), .b(new_n213), .c(new_n177), .d(new_n209), .o1(new_n223));
  aoi012aa1n03x5               g128(.a(new_n221), .b(new_n223), .c(new_n219), .o1(new_n224));
  nor002aa1n02x5               g129(.a(new_n224), .b(new_n222), .o1(\s[22] ));
  nano22aa1n03x7               g130(.a(new_n221), .b(new_n219), .c(new_n216), .out0(new_n226));
  and003aa1n02x5               g131(.a(new_n186), .b(new_n226), .c(new_n207), .o(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n185), .c(new_n125), .d(new_n171), .o1(new_n228));
  oao003aa1n12x5               g133(.a(\a[22] ), .b(\b[21] ), .c(new_n219), .carry(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoi012aa1n02x5               g135(.a(new_n230), .b(new_n213), .c(new_n226), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[22] ), .b(\a[23] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  xnbna2aa1n03x5               g138(.a(new_n233), .b(new_n228), .c(new_n231), .out0(\s[23] ));
  nor042aa1n06x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  tech160nm_fiaoi012aa1n02p5x5 g141(.a(new_n232), .b(new_n228), .c(new_n231), .o1(new_n237));
  tech160nm_fixnrc02aa1n04x5   g142(.a(\b[23] ), .b(\a[24] ), .out0(new_n238));
  nano22aa1n02x4               g143(.a(new_n237), .b(new_n236), .c(new_n238), .out0(new_n239));
  inv030aa1n02x5               g144(.a(new_n231), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n233), .b(new_n240), .c(new_n177), .d(new_n227), .o1(new_n241));
  aoi012aa1n03x5               g146(.a(new_n238), .b(new_n241), .c(new_n236), .o1(new_n242));
  nor002aa1n02x5               g147(.a(new_n242), .b(new_n239), .o1(\s[24] ));
  nor042aa1n03x5               g148(.a(new_n238), .b(new_n232), .o1(new_n244));
  nano22aa1n03x7               g149(.a(new_n208), .b(new_n226), .c(new_n244), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n185), .c(new_n125), .d(new_n171), .o1(new_n246));
  inv020aa1n04x5               g151(.a(new_n212), .o1(new_n247));
  aoai13aa1n06x5               g152(.a(new_n226), .b(new_n247), .c(new_n207), .d(new_n203), .o1(new_n248));
  inv040aa1n02x5               g153(.a(new_n244), .o1(new_n249));
  oao003aa1n02x5               g154(.a(\a[24] ), .b(\b[23] ), .c(new_n236), .carry(new_n250));
  aoai13aa1n12x5               g155(.a(new_n250), .b(new_n249), .c(new_n248), .d(new_n229), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  xnrc02aa1n12x5               g157(.a(\b[24] ), .b(\a[25] ), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xnbna2aa1n03x5               g159(.a(new_n254), .b(new_n246), .c(new_n252), .out0(\s[25] ));
  nor042aa1n03x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  tech160nm_fiaoi012aa1n04x5   g162(.a(new_n253), .b(new_n246), .c(new_n252), .o1(new_n258));
  xnrc02aa1n02x5               g163(.a(\b[25] ), .b(\a[26] ), .out0(new_n259));
  nano22aa1n02x4               g164(.a(new_n258), .b(new_n257), .c(new_n259), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n254), .b(new_n251), .c(new_n177), .d(new_n245), .o1(new_n261));
  aoi012aa1n03x5               g166(.a(new_n259), .b(new_n261), .c(new_n257), .o1(new_n262));
  nor002aa1n02x5               g167(.a(new_n262), .b(new_n260), .o1(\s[26] ));
  nor042aa1n06x5               g168(.a(new_n259), .b(new_n253), .o1(new_n264));
  nano32aa1n03x7               g169(.a(new_n208), .b(new_n264), .c(new_n226), .d(new_n244), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n185), .c(new_n125), .d(new_n171), .o1(new_n266));
  oao003aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .c(new_n257), .carry(new_n267));
  aobi12aa1n12x5               g172(.a(new_n267), .b(new_n251), .c(new_n264), .out0(new_n268));
  xorc02aa1n12x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  xnbna2aa1n06x5               g174(.a(new_n269), .b(new_n266), .c(new_n268), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  inv040aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  aobi12aa1n06x5               g177(.a(new_n269), .b(new_n266), .c(new_n268), .out0(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  nano22aa1n03x7               g179(.a(new_n273), .b(new_n272), .c(new_n274), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n244), .b(new_n230), .c(new_n213), .d(new_n226), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n264), .o1(new_n277));
  aoai13aa1n04x5               g182(.a(new_n267), .b(new_n277), .c(new_n276), .d(new_n250), .o1(new_n278));
  aoai13aa1n02x7               g183(.a(new_n269), .b(new_n278), .c(new_n177), .d(new_n265), .o1(new_n279));
  aoi012aa1n03x5               g184(.a(new_n274), .b(new_n279), .c(new_n272), .o1(new_n280));
  norp02aa1n03x5               g185(.a(new_n280), .b(new_n275), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n269), .b(new_n274), .out0(new_n282));
  aobi12aa1n06x5               g187(.a(new_n282), .b(new_n266), .c(new_n268), .out0(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n272), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  nano22aa1n03x7               g190(.a(new_n283), .b(new_n284), .c(new_n285), .out0(new_n286));
  aoai13aa1n03x5               g191(.a(new_n282), .b(new_n278), .c(new_n177), .d(new_n265), .o1(new_n287));
  aoi012aa1n03x5               g192(.a(new_n285), .b(new_n287), .c(new_n284), .o1(new_n288));
  norp02aa1n03x5               g193(.a(new_n288), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n269), .b(new_n285), .c(new_n274), .out0(new_n291));
  aobi12aa1n06x5               g196(.a(new_n291), .b(new_n266), .c(new_n268), .out0(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  nano22aa1n03x7               g199(.a(new_n292), .b(new_n293), .c(new_n294), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n291), .b(new_n278), .c(new_n177), .d(new_n265), .o1(new_n296));
  aoi012aa1n03x5               g201(.a(new_n294), .b(new_n296), .c(new_n293), .o1(new_n297));
  norp02aa1n03x5               g202(.a(new_n297), .b(new_n295), .o1(\s[30] ));
  xnrc02aa1n02x5               g203(.a(\b[30] ), .b(\a[31] ), .out0(new_n299));
  norb02aa1n02x7               g204(.a(new_n291), .b(new_n294), .out0(new_n300));
  aobi12aa1n06x5               g205(.a(new_n300), .b(new_n266), .c(new_n268), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n302));
  nano22aa1n03x7               g207(.a(new_n301), .b(new_n299), .c(new_n302), .out0(new_n303));
  aoai13aa1n02x7               g208(.a(new_n300), .b(new_n278), .c(new_n177), .d(new_n265), .o1(new_n304));
  aoi012aa1n03x5               g209(.a(new_n299), .b(new_n304), .c(new_n302), .o1(new_n305));
  nor002aa1n02x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  xnbna2aa1n03x5               g211(.a(new_n101), .b(new_n104), .c(new_n107), .out0(\s[3] ));
  oaoi03aa1n02x5               g212(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g214(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g215(.a(\a[6] ), .o1(new_n311));
  oaoi03aa1n03x5               g216(.a(new_n120), .b(new_n121), .c(new_n109), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(new_n311), .out0(\s[6] ));
  inv000aa1d42x5               g218(.a(\b[5] ), .o1(new_n314));
  norb02aa1n02x5               g219(.a(new_n114), .b(new_n113), .out0(new_n315));
  nanp02aa1n02x5               g220(.a(new_n312), .b(new_n110), .o1(new_n316));
  oai112aa1n03x5               g221(.a(new_n316), .b(new_n315), .c(new_n314), .d(new_n311), .o1(new_n317));
  oaoi13aa1n02x5               g222(.a(new_n315), .b(new_n316), .c(new_n311), .d(new_n314), .o1(new_n318));
  norb02aa1n02x5               g223(.a(new_n317), .b(new_n318), .out0(\s[7] ));
  oai012aa1n02x5               g224(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g226(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


