// Benchmark "adder" written by ABC on Thu Jul 18 12:33:45 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n317, new_n319, new_n320, new_n321, new_n322, new_n324, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n332, new_n334;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  inv000aa1d42x5               g003(.a(\a[3] ), .o1(new_n99));
  nanb02aa1n02x5               g004(.a(\b[2] ), .b(new_n99), .out0(new_n100));
  oao003aa1n02x5               g005(.a(\a[4] ), .b(\b[3] ), .c(new_n100), .carry(new_n101));
  tech160nm_fixorc02aa1n04x5   g006(.a(\a[4] ), .b(\b[3] ), .out0(new_n102));
  inv000aa1d42x5               g007(.a(\a[2] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(\b[1] ), .o1(new_n104));
  nand02aa1n06x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  oao003aa1n03x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .carry(new_n106));
  tech160nm_fixorc02aa1n04x5   g011(.a(\a[3] ), .b(\b[2] ), .out0(new_n107));
  nanp03aa1n06x5               g012(.a(new_n106), .b(new_n102), .c(new_n107), .o1(new_n108));
  tech160nm_fixorc02aa1n03p5x5 g013(.a(\a[8] ), .b(\b[7] ), .out0(new_n109));
  inv000aa1d42x5               g014(.a(\a[5] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\a[7] ), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\b[4] ), .o1(new_n112));
  aboi22aa1n03x5               g017(.a(\b[6] ), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n113));
  aoi022aa1d24x5               g018(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n114));
  oai122aa1n03x5               g019(.a(new_n114), .b(\a[6] ), .c(\b[5] ), .d(new_n110), .e(new_n112), .o1(new_n115));
  nanb03aa1n09x5               g020(.a(new_n115), .b(new_n109), .c(new_n113), .out0(new_n116));
  inv000aa1d42x5               g021(.a(\a[8] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\b[7] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[6] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n111), .o1(new_n120));
  oai022aa1d18x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  aob012aa1n02x5               g026(.a(new_n120), .b(new_n121), .c(new_n114), .out0(new_n122));
  tech160nm_fioaoi03aa1n03p5x5 g027(.a(new_n117), .b(new_n118), .c(new_n122), .o1(new_n123));
  aoai13aa1n12x5               g028(.a(new_n123), .b(new_n116), .c(new_n108), .d(new_n101), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  nand23aa1n03x5               g030(.a(new_n124), .b(new_n98), .c(new_n125), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[10] ), .b(\b[9] ), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  xnrc02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .out0(new_n129));
  inv000aa1d42x5               g034(.a(\a[10] ), .o1(new_n130));
  inv000aa1d42x5               g035(.a(\b[9] ), .o1(new_n131));
  aboi22aa1d24x5               g036(.a(\b[8] ), .b(new_n97), .c(new_n131), .d(new_n130), .out0(new_n132));
  nanp02aa1n06x5               g037(.a(new_n126), .b(new_n132), .o1(new_n133));
  oaib12aa1n02x5               g038(.a(new_n133), .b(new_n131), .c(\a[10] ), .out0(new_n134));
  nor002aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  aoi022aa1d24x5               g040(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoi022aa1n02x5               g042(.a(new_n134), .b(new_n129), .c(new_n133), .d(new_n137), .o1(\s[11] ));
  tech160nm_fiaoi012aa1n05x5   g043(.a(new_n135), .b(new_n133), .c(new_n136), .o1(new_n139));
  xorc02aa1n02x5               g044(.a(\a[12] ), .b(\b[11] ), .out0(new_n140));
  inv000aa1d42x5               g045(.a(new_n140), .o1(new_n141));
  aoi112aa1n03x5               g046(.a(new_n140), .b(new_n135), .c(new_n133), .d(new_n136), .o1(new_n142));
  oab012aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n141), .out0(\s[12] ));
  and002aa1n02x5               g048(.a(\b[8] ), .b(\a[9] ), .o(new_n144));
  inv000aa1n02x5               g049(.a(new_n136), .o1(new_n145));
  norp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand42aa1n06x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb03aa1n06x5               g052(.a(new_n147), .b(new_n135), .c(new_n146), .out0(new_n148));
  nona23aa1d18x5               g053(.a(new_n132), .b(new_n148), .c(new_n145), .d(new_n144), .out0(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  oa0012aa1n06x5               g055(.a(new_n148), .b(new_n145), .c(new_n132), .o(new_n151));
  norb02aa1n02x7               g056(.a(new_n147), .b(new_n151), .out0(new_n152));
  tech160nm_fiao0012aa1n02p5x5 g057(.a(new_n152), .b(new_n124), .c(new_n150), .o(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  xnrc02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  nanb02aa1n02x5               g060(.a(new_n155), .b(new_n153), .out0(new_n156));
  xnrc02aa1n12x5               g061(.a(\b[13] ), .b(\a[14] ), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  norp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n157), .b(new_n159), .out0(new_n160));
  aoi012aa1n02x5               g065(.a(new_n152), .b(new_n124), .c(new_n150), .o1(new_n161));
  oaoi03aa1n02x5               g066(.a(\a[13] ), .b(\b[12] ), .c(new_n161), .o1(new_n162));
  aoi022aa1n02x5               g067(.a(new_n156), .b(new_n160), .c(new_n162), .d(new_n158), .o1(\s[14] ));
  nor042aa1n04x5               g068(.a(new_n157), .b(new_n155), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n152), .c(new_n124), .d(new_n150), .o1(new_n165));
  oai022aa1n03x5               g070(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n166));
  aob012aa1n02x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(new_n167));
  xorc02aa1n02x5               g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n165), .c(new_n167), .out0(\s[15] ));
  aob012aa1n03x5               g074(.a(new_n168), .b(new_n165), .c(new_n167), .out0(new_n170));
  xorc02aa1n02x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  inv000aa1d42x5               g076(.a(\a[15] ), .o1(new_n172));
  inv000aa1d42x5               g077(.a(\b[14] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(new_n173), .b(new_n172), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n171), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n168), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n174), .b(new_n176), .c(new_n165), .d(new_n167), .o1(new_n177));
  aoi022aa1n03x5               g082(.a(new_n177), .b(new_n171), .c(new_n170), .d(new_n175), .o1(\s[16] ));
  inv000aa1d42x5               g083(.a(\a[16] ), .o1(new_n179));
  xroi22aa1d04x5               g084(.a(new_n172), .b(\b[14] ), .c(new_n179), .d(\b[15] ), .out0(new_n180));
  nano22aa1d15x5               g085(.a(new_n149), .b(new_n180), .c(new_n164), .out0(new_n181));
  nand02aa1d04x5               g086(.a(new_n124), .b(new_n181), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[15] ), .o1(new_n183));
  aoi022aa1n02x5               g088(.a(new_n183), .b(new_n179), .c(\a[12] ), .d(\b[11] ), .o1(new_n184));
  nand23aa1n03x5               g089(.a(new_n180), .b(new_n164), .c(new_n184), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(new_n183), .b(new_n179), .o1(new_n186));
  oai022aa1n02x5               g091(.a(new_n172), .b(new_n173), .c(new_n183), .d(new_n179), .o1(new_n187));
  aoai13aa1n03x5               g092(.a(new_n186), .b(new_n187), .c(new_n167), .d(new_n174), .o1(new_n188));
  oab012aa1n04x5               g093(.a(new_n188), .b(new_n185), .c(new_n151), .out0(new_n189));
  tech160nm_fixorc02aa1n04x5   g094(.a(\a[17] ), .b(\b[16] ), .out0(new_n190));
  xnbna2aa1n03x5               g095(.a(new_n190), .b(new_n182), .c(new_n189), .out0(\s[17] ));
  inv000aa1d42x5               g096(.a(\a[17] ), .o1(new_n192));
  nanb02aa1n02x5               g097(.a(\b[16] ), .b(new_n192), .out0(new_n193));
  oabi12aa1n12x5               g098(.a(new_n188), .b(new_n185), .c(new_n151), .out0(new_n194));
  aoai13aa1n02x5               g099(.a(new_n190), .b(new_n194), .c(new_n124), .d(new_n181), .o1(new_n195));
  tech160nm_fixorc02aa1n04x5   g100(.a(\a[18] ), .b(\b[17] ), .out0(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n196), .b(new_n195), .c(new_n193), .out0(\s[18] ));
  inv000aa1d42x5               g102(.a(\a[18] ), .o1(new_n198));
  xroi22aa1d04x5               g103(.a(new_n192), .b(\b[16] ), .c(new_n198), .d(\b[17] ), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n194), .c(new_n124), .d(new_n181), .o1(new_n200));
  oaih22aa1n04x5               g105(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n201));
  oaib12aa1n12x5               g106(.a(new_n201), .b(new_n198), .c(\b[17] ), .out0(new_n202));
  nor002aa1d32x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand42aa1n08x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n200), .c(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand02aa1d04x5               g113(.a(new_n182), .b(new_n189), .o1(new_n209));
  tech160nm_fioaoi03aa1n03p5x5 g114(.a(\a[18] ), .b(\b[17] ), .c(new_n193), .o1(new_n210));
  aoai13aa1n02x5               g115(.a(new_n206), .b(new_n210), .c(new_n209), .d(new_n199), .o1(new_n211));
  nor002aa1n06x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand42aa1n08x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  aoib12aa1n02x5               g119(.a(new_n203), .b(new_n213), .c(new_n212), .out0(new_n215));
  inv000aa1n02x5               g120(.a(new_n203), .o1(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n205), .c(new_n200), .d(new_n202), .o1(new_n217));
  aoi022aa1n03x5               g122(.a(new_n217), .b(new_n214), .c(new_n211), .d(new_n215), .o1(\s[20] ));
  nano23aa1n09x5               g123(.a(new_n203), .b(new_n212), .c(new_n213), .d(new_n204), .out0(new_n219));
  nand23aa1n06x5               g124(.a(new_n219), .b(new_n190), .c(new_n196), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n194), .c(new_n124), .d(new_n181), .o1(new_n222));
  nona23aa1d24x5               g127(.a(new_n213), .b(new_n204), .c(new_n203), .d(new_n212), .out0(new_n223));
  oaoi03aa1n12x5               g128(.a(\a[20] ), .b(\b[19] ), .c(new_n216), .o1(new_n224));
  oabi12aa1n18x5               g129(.a(new_n224), .b(new_n223), .c(new_n202), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[20] ), .b(\a[21] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  xnbna2aa1n03x5               g133(.a(new_n228), .b(new_n222), .c(new_n226), .out0(\s[21] ));
  aoai13aa1n02x5               g134(.a(new_n228), .b(new_n225), .c(new_n209), .d(new_n221), .o1(new_n230));
  xnrc02aa1n03x5               g135(.a(\b[21] ), .b(\a[22] ), .out0(new_n231));
  nor042aa1n09x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n231), .b(new_n232), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n232), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n227), .c(new_n222), .d(new_n226), .o1(new_n235));
  aboi22aa1n03x5               g140(.a(new_n231), .b(new_n235), .c(new_n230), .d(new_n233), .out0(\s[22] ));
  nor042aa1n06x5               g141(.a(new_n231), .b(new_n227), .o1(new_n237));
  norb02aa1n06x4               g142(.a(new_n237), .b(new_n220), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n194), .c(new_n124), .d(new_n181), .o1(new_n239));
  oao003aa1n12x5               g144(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .carry(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n241), .b(new_n225), .c(new_n237), .o1(new_n242));
  nand22aa1n04x5               g147(.a(new_n239), .b(new_n242), .o1(new_n243));
  xorc02aa1n12x5               g148(.a(\a[23] ), .b(\b[22] ), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n244), .b(new_n241), .c(new_n225), .d(new_n237), .o1(new_n245));
  aoi022aa1n02x5               g150(.a(new_n243), .b(new_n244), .c(new_n239), .d(new_n245), .o1(\s[23] ));
  xorc02aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .out0(new_n247));
  nor042aa1n06x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  norp02aa1n02x5               g153(.a(new_n247), .b(new_n248), .o1(new_n249));
  aobi12aa1n03x5               g154(.a(new_n249), .b(new_n243), .c(new_n244), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n248), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n244), .o1(new_n252));
  aoai13aa1n02x5               g157(.a(new_n251), .b(new_n252), .c(new_n239), .d(new_n242), .o1(new_n253));
  aoi012aa1n03x5               g158(.a(new_n250), .b(new_n253), .c(new_n247), .o1(\s[24] ));
  nano32aa1n03x7               g159(.a(new_n220), .b(new_n247), .c(new_n237), .d(new_n244), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n194), .c(new_n124), .d(new_n181), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n237), .b(new_n224), .c(new_n219), .d(new_n210), .o1(new_n257));
  and002aa1n06x5               g162(.a(new_n247), .b(new_n244), .o(new_n258));
  inv020aa1n04x5               g163(.a(new_n258), .o1(new_n259));
  oao003aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .c(new_n251), .carry(new_n260));
  aoai13aa1n12x5               g165(.a(new_n260), .b(new_n259), .c(new_n257), .d(new_n240), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  xorc02aa1n12x5               g167(.a(\a[25] ), .b(\b[24] ), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n256), .c(new_n262), .out0(\s[25] ));
  aoai13aa1n03x5               g169(.a(new_n263), .b(new_n261), .c(new_n209), .d(new_n255), .o1(new_n265));
  xorc02aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .out0(new_n266));
  nor042aa1n03x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  norp02aa1n02x5               g172(.a(new_n266), .b(new_n267), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n267), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n263), .o1(new_n270));
  aoai13aa1n02x5               g175(.a(new_n269), .b(new_n270), .c(new_n256), .d(new_n262), .o1(new_n271));
  aoi022aa1n03x5               g176(.a(new_n271), .b(new_n266), .c(new_n265), .d(new_n268), .o1(\s[26] ));
  and002aa1n06x5               g177(.a(new_n266), .b(new_n263), .o(new_n273));
  inv000aa1n02x5               g178(.a(new_n273), .o1(new_n274));
  nano23aa1n06x5               g179(.a(new_n274), .b(new_n220), .c(new_n258), .d(new_n237), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n194), .c(new_n124), .d(new_n181), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n277));
  inv000aa1d42x5               g182(.a(new_n277), .o1(new_n278));
  aoi012aa1n12x5               g183(.a(new_n278), .b(new_n261), .c(new_n273), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n279), .c(new_n276), .out0(\s[27] ));
  aoai13aa1n04x5               g186(.a(new_n258), .b(new_n241), .c(new_n225), .d(new_n237), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n277), .b(new_n274), .c(new_n282), .d(new_n260), .o1(new_n283));
  aoai13aa1n02x5               g188(.a(new_n280), .b(new_n283), .c(new_n209), .d(new_n275), .o1(new_n284));
  tech160nm_fixorc02aa1n03p5x5 g189(.a(\a[28] ), .b(\b[27] ), .out0(new_n285));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n285), .b(new_n286), .o1(new_n287));
  inv000aa1n03x5               g192(.a(new_n286), .o1(new_n288));
  inv000aa1n02x5               g193(.a(new_n280), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n288), .b(new_n289), .c(new_n279), .d(new_n276), .o1(new_n290));
  aoi022aa1n03x5               g195(.a(new_n290), .b(new_n285), .c(new_n284), .d(new_n287), .o1(\s[28] ));
  and002aa1n02x5               g196(.a(new_n285), .b(new_n280), .o(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n283), .c(new_n209), .d(new_n275), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n292), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .carry(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n294), .c(new_n279), .d(new_n276), .o1(new_n296));
  tech160nm_fixorc02aa1n03p5x5 g201(.a(\a[29] ), .b(\b[28] ), .out0(new_n297));
  norb02aa1n02x5               g202(.a(new_n295), .b(new_n297), .out0(new_n298));
  aoi022aa1n03x5               g203(.a(new_n296), .b(new_n297), .c(new_n293), .d(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g205(.a(new_n289), .b(new_n285), .c(new_n297), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n283), .c(new_n209), .d(new_n275), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n301), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .carry(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n303), .c(new_n279), .d(new_n276), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .out0(new_n306));
  norb02aa1n02x5               g211(.a(new_n304), .b(new_n306), .out0(new_n307));
  aoi022aa1n03x5               g212(.a(new_n305), .b(new_n306), .c(new_n302), .d(new_n307), .o1(\s[30] ));
  nano32aa1n06x5               g213(.a(new_n289), .b(new_n306), .c(new_n285), .d(new_n297), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n283), .c(new_n209), .d(new_n275), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[31] ), .b(\b[30] ), .out0(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n312));
  norb02aa1n02x5               g217(.a(new_n312), .b(new_n311), .out0(new_n313));
  inv000aa1d42x5               g218(.a(new_n309), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n312), .b(new_n314), .c(new_n279), .d(new_n276), .o1(new_n315));
  aoi022aa1n03x5               g220(.a(new_n315), .b(new_n311), .c(new_n310), .d(new_n313), .o1(\s[31] ));
  oaoi03aa1n02x5               g221(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[2] ), .c(new_n99), .out0(\s[3] ));
  nanp02aa1n02x5               g223(.a(new_n107), .b(new_n102), .o1(new_n319));
  oai012aa1n02x5               g224(.a(new_n101), .b(new_n319), .c(new_n317), .o1(new_n320));
  norp02aa1n02x5               g225(.a(\b[2] ), .b(\a[3] ), .o1(new_n321));
  aoi112aa1n02x5               g226(.a(new_n321), .b(new_n102), .c(new_n106), .d(new_n107), .o1(new_n322));
  oaoi13aa1n02x5               g227(.a(new_n322), .b(new_n320), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorc02aa1n02x5               g228(.a(\a[5] ), .b(\b[4] ), .out0(new_n324));
  xnbna2aa1n03x5               g229(.a(new_n324), .b(new_n108), .c(new_n101), .out0(\s[5] ));
  xorc02aa1n02x5               g230(.a(\a[6] ), .b(\b[5] ), .out0(new_n326));
  oaoi03aa1n02x5               g231(.a(new_n110), .b(new_n112), .c(new_n320), .o1(new_n327));
  nanp02aa1n02x5               g232(.a(\b[5] ), .b(\a[6] ), .o1(new_n328));
  norb02aa1n02x5               g233(.a(new_n328), .b(new_n121), .out0(new_n329));
  aob012aa1n02x5               g234(.a(new_n329), .b(new_n320), .c(new_n324), .out0(new_n330));
  oai012aa1n02x5               g235(.a(new_n330), .b(new_n327), .c(new_n326), .o1(\s[6] ));
  xorc02aa1n02x5               g236(.a(\a[7] ), .b(\b[6] ), .out0(new_n332));
  xobna2aa1n03x5               g237(.a(new_n332), .b(new_n330), .c(new_n328), .out0(\s[7] ));
  nanp03aa1n02x5               g238(.a(new_n330), .b(new_n328), .c(new_n332), .o1(new_n334));
  xnbna2aa1n03x5               g239(.a(new_n109), .b(new_n334), .c(new_n120), .out0(\s[8] ));
  xorb03aa1n02x5               g240(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


