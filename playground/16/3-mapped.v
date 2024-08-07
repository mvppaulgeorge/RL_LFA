// Benchmark "adder" written by ABC on Wed Jul 17 20:08:49 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n322, new_n323, new_n325, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  nor002aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n04x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n05x5   g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  tech160nm_fixnrc02aa1n04x5   g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  xnrc02aa1n12x5               g008(.a(\b[2] ), .b(\a[3] ), .out0(new_n104));
  nor002aa1d32x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(new_n105), .o1(new_n106));
  oao003aa1n02x5               g011(.a(\a[4] ), .b(\b[3] ), .c(new_n106), .carry(new_n107));
  oai013aa1n09x5               g012(.a(new_n107), .b(new_n103), .c(new_n104), .d(new_n102), .o1(new_n108));
  xnrc02aa1n12x5               g013(.a(\b[7] ), .b(\a[8] ), .out0(new_n109));
  xnrc02aa1n12x5               g014(.a(\b[6] ), .b(\a[7] ), .out0(new_n110));
  nor002aa1d32x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nor002aa1n10x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand22aa1n09x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nona23aa1n06x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nor043aa1n03x5               g020(.a(new_n115), .b(new_n110), .c(new_n109), .o1(new_n116));
  aoi012aa1n02x7               g021(.a(new_n113), .b(new_n111), .c(new_n114), .o1(new_n117));
  nor002aa1d32x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(new_n118), .o1(new_n119));
  oao003aa1n02x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .carry(new_n120));
  oai013aa1n03x5               g025(.a(new_n120), .b(new_n110), .c(new_n109), .d(new_n117), .o1(new_n121));
  tech160nm_fixorc02aa1n03p5x5 g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n123));
  nor042aa1n03x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nand42aa1n06x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norb02aa1n09x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n98), .out0(\s[10] ));
  aoi012aa1n06x5               g032(.a(new_n121), .b(new_n108), .c(new_n116), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(new_n122), .b(new_n126), .o1(new_n129));
  inv000aa1d42x5               g034(.a(\b[8] ), .o1(new_n130));
  aoai13aa1n04x5               g035(.a(new_n125), .b(new_n124), .c(new_n97), .d(new_n130), .o1(new_n131));
  tech160nm_fioai012aa1n05x5   g036(.a(new_n131), .b(new_n128), .c(new_n129), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  norp02aa1n06x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand42aa1n08x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(new_n139));
  aoai13aa1n02x5               g044(.a(new_n139), .b(new_n134), .c(new_n132), .d(new_n136), .o1(new_n140));
  nona22aa1n02x4               g045(.a(new_n138), .b(new_n137), .c(new_n134), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n140), .b(new_n141), .c(new_n136), .d(new_n132), .o1(\s[12] ));
  nano23aa1n06x5               g047(.a(new_n134), .b(new_n137), .c(new_n138), .d(new_n135), .out0(new_n143));
  nand23aa1n04x5               g048(.a(new_n143), .b(new_n122), .c(new_n126), .o1(new_n144));
  nona23aa1n09x5               g049(.a(new_n138), .b(new_n135), .c(new_n134), .d(new_n137), .out0(new_n145));
  oai012aa1n02x5               g050(.a(new_n138), .b(new_n137), .c(new_n134), .o1(new_n146));
  tech160nm_fioai012aa1n04x5   g051(.a(new_n146), .b(new_n145), .c(new_n131), .o1(new_n147));
  oabi12aa1n06x5               g052(.a(new_n147), .b(new_n128), .c(new_n144), .out0(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n08x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nand42aa1n10x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nano22aa1n02x4               g056(.a(new_n150), .b(new_n148), .c(new_n151), .out0(new_n152));
  norp02aa1n12x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nand42aa1n08x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n150), .c(new_n148), .d(new_n151), .o1(new_n156));
  nona22aa1n02x4               g061(.a(new_n154), .b(new_n153), .c(new_n150), .out0(new_n157));
  oai012aa1n02x5               g062(.a(new_n156), .b(new_n152), .c(new_n157), .o1(\s[14] ));
  nona23aa1n02x4               g063(.a(new_n154), .b(new_n151), .c(new_n150), .d(new_n153), .out0(new_n159));
  nano23aa1n03x7               g064(.a(new_n150), .b(new_n153), .c(new_n154), .d(new_n151), .out0(new_n160));
  aoi022aa1n02x5               g065(.a(new_n147), .b(new_n160), .c(new_n154), .d(new_n157), .o1(new_n161));
  oai013aa1n03x5               g066(.a(new_n161), .b(new_n128), .c(new_n144), .d(new_n159), .o1(new_n162));
  xorb03aa1n03x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand02aa1d04x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  aoi012aa1n02x5               g070(.a(new_n164), .b(new_n162), .c(new_n165), .o1(new_n166));
  nor042aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  tech160nm_finand02aa1n03p5x5 g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  norb02aa1n02x5               g074(.a(new_n165), .b(new_n164), .out0(new_n170));
  nona22aa1n02x4               g075(.a(new_n168), .b(new_n167), .c(new_n164), .out0(new_n171));
  tech160nm_fiao0012aa1n03p5x5 g076(.a(new_n171), .b(new_n162), .c(new_n170), .o(new_n172));
  oaih12aa1n02x5               g077(.a(new_n172), .b(new_n166), .c(new_n169), .o1(\s[16] ));
  nano23aa1n06x5               g078(.a(new_n164), .b(new_n167), .c(new_n168), .d(new_n165), .out0(new_n174));
  nano22aa1n09x5               g079(.a(new_n144), .b(new_n160), .c(new_n174), .out0(new_n175));
  aoai13aa1n12x5               g080(.a(new_n175), .b(new_n121), .c(new_n108), .d(new_n116), .o1(new_n176));
  nano22aa1n03x7               g081(.a(new_n159), .b(new_n170), .c(new_n169), .out0(new_n177));
  oa0012aa1n02x5               g082(.a(new_n154), .b(new_n153), .c(new_n150), .o(new_n178));
  tech160nm_fiao0012aa1n02p5x5 g083(.a(new_n167), .b(new_n164), .c(new_n168), .o(new_n179));
  tech160nm_fiaoi012aa1n04x5   g084(.a(new_n179), .b(new_n174), .c(new_n178), .o1(new_n180));
  aobi12aa1n12x5               g085(.a(new_n180), .b(new_n177), .c(new_n147), .out0(new_n181));
  xorc02aa1n02x5               g086(.a(\a[17] ), .b(\b[16] ), .out0(new_n182));
  xnbna2aa1n03x5               g087(.a(new_n182), .b(new_n176), .c(new_n181), .out0(\s[17] ));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  nanb02aa1d24x5               g089(.a(\b[16] ), .b(new_n184), .out0(new_n185));
  inv000aa1d42x5               g090(.a(new_n104), .o1(new_n186));
  nona22aa1n03x5               g091(.a(new_n186), .b(new_n103), .c(new_n102), .out0(new_n187));
  inv000aa1d42x5               g092(.a(new_n109), .o1(new_n188));
  inv000aa1d42x5               g093(.a(new_n110), .o1(new_n189));
  nanb03aa1n02x5               g094(.a(new_n115), .b(new_n188), .c(new_n189), .out0(new_n190));
  norp03aa1n03x5               g095(.a(new_n109), .b(new_n110), .c(new_n117), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n120), .b(new_n191), .out0(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n190), .c(new_n187), .d(new_n107), .o1(new_n193));
  aob012aa1n06x5               g098(.a(new_n180), .b(new_n147), .c(new_n177), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n182), .b(new_n194), .c(new_n193), .d(new_n175), .o1(new_n195));
  xnrc02aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .out0(new_n196));
  xobna2aa1n03x5               g101(.a(new_n196), .b(new_n195), .c(new_n185), .out0(\s[18] ));
  inv000aa1d42x5               g102(.a(\a[18] ), .o1(new_n198));
  xroi22aa1d04x5               g103(.a(new_n184), .b(\b[16] ), .c(new_n198), .d(\b[17] ), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n194), .c(new_n193), .d(new_n175), .o1(new_n200));
  nor042aa1n04x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nand02aa1n03x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  oaoi03aa1n12x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n185), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n204), .b(new_n200), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g113(.a(new_n199), .o1(new_n209));
  aoai13aa1n03x5               g114(.a(new_n206), .b(new_n209), .c(new_n176), .d(new_n181), .o1(new_n210));
  nor042aa1n03x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nand42aa1n04x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nanb02aa1n02x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  aoai13aa1n03x5               g118(.a(new_n213), .b(new_n201), .c(new_n210), .d(new_n204), .o1(new_n214));
  norb03aa1n02x5               g119(.a(new_n212), .b(new_n201), .c(new_n211), .out0(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n203), .c(new_n200), .d(new_n206), .o1(new_n216));
  nanp02aa1n03x5               g121(.a(new_n214), .b(new_n216), .o1(\s[20] ));
  nano23aa1n09x5               g122(.a(new_n201), .b(new_n211), .c(new_n212), .d(new_n202), .out0(new_n218));
  nanb03aa1n06x5               g123(.a(new_n196), .b(new_n218), .c(new_n182), .out0(new_n219));
  oaih12aa1n02x5               g124(.a(new_n212), .b(new_n211), .c(new_n201), .o1(new_n220));
  aobi12aa1n02x5               g125(.a(new_n220), .b(new_n218), .c(new_n205), .out0(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n219), .c(new_n176), .d(new_n181), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n09x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  nand42aa1d28x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  nor042aa1n06x5               g131(.a(\b[21] ), .b(\a[22] ), .o1(new_n227));
  nand42aa1d28x5               g132(.a(\b[21] ), .b(\a[22] ), .o1(new_n228));
  nanb02aa1n02x5               g133(.a(new_n227), .b(new_n228), .out0(new_n229));
  aoai13aa1n03x5               g134(.a(new_n229), .b(new_n224), .c(new_n222), .d(new_n226), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(new_n222), .b(new_n226), .o1(new_n231));
  norb03aa1n02x5               g136(.a(new_n228), .b(new_n224), .c(new_n227), .out0(new_n232));
  nand42aa1n02x5               g137(.a(new_n231), .b(new_n232), .o1(new_n233));
  nanp02aa1n03x5               g138(.a(new_n230), .b(new_n233), .o1(\s[22] ));
  nano23aa1d15x5               g139(.a(new_n224), .b(new_n227), .c(new_n228), .d(new_n225), .out0(new_n235));
  nanp03aa1n02x5               g140(.a(new_n199), .b(new_n218), .c(new_n235), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n235), .o1(new_n237));
  tech160nm_fiao0012aa1n02p5x5 g142(.a(new_n227), .b(new_n224), .c(new_n228), .o(new_n238));
  oab012aa1n02x4               g143(.a(new_n238), .b(new_n221), .c(new_n237), .out0(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n236), .c(new_n176), .d(new_n181), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1d32x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  nanp02aa1n04x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  nor002aa1d32x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  nand02aa1n16x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n248), .b(new_n242), .c(new_n240), .d(new_n244), .o1(new_n249));
  nand02aa1n02x5               g154(.a(new_n240), .b(new_n244), .o1(new_n250));
  nona22aa1d36x5               g155(.a(new_n246), .b(new_n245), .c(new_n242), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  nand02aa1n02x5               g157(.a(new_n250), .b(new_n252), .o1(new_n253));
  nanp02aa1n03x5               g158(.a(new_n249), .b(new_n253), .o1(\s[24] ));
  nano23aa1n09x5               g159(.a(new_n242), .b(new_n245), .c(new_n246), .d(new_n243), .out0(new_n255));
  nand22aa1n09x5               g160(.a(new_n255), .b(new_n235), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  nand23aa1n02x5               g162(.a(new_n257), .b(new_n199), .c(new_n218), .o1(new_n258));
  nand02aa1n04x5               g163(.a(new_n218), .b(new_n205), .o1(new_n259));
  aoi022aa1n06x5               g164(.a(new_n255), .b(new_n238), .c(new_n246), .d(new_n251), .o1(new_n260));
  aoai13aa1n12x5               g165(.a(new_n260), .b(new_n256), .c(new_n259), .d(new_n220), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n04x5               g167(.a(new_n262), .b(new_n258), .c(new_n176), .d(new_n181), .o1(new_n263));
  xorb03aa1n02x5               g168(.a(new_n263), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  xorc02aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .out0(new_n266));
  xnrc02aa1n02x5               g171(.a(\b[25] ), .b(\a[26] ), .out0(new_n267));
  aoai13aa1n03x5               g172(.a(new_n267), .b(new_n265), .c(new_n263), .d(new_n266), .o1(new_n268));
  nand02aa1n02x5               g173(.a(new_n263), .b(new_n266), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n267), .b(new_n265), .o1(new_n270));
  nanp02aa1n03x5               g175(.a(new_n269), .b(new_n270), .o1(new_n271));
  nanp02aa1n03x5               g176(.a(new_n268), .b(new_n271), .o1(\s[26] ));
  norb02aa1n02x5               g177(.a(new_n266), .b(new_n267), .out0(new_n273));
  norb03aa1n03x5               g178(.a(new_n273), .b(new_n219), .c(new_n256), .out0(new_n274));
  inv000aa1n02x5               g179(.a(new_n274), .o1(new_n275));
  oaoi03aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n270), .o1(new_n276));
  aoi012aa1n12x5               g181(.a(new_n276), .b(new_n261), .c(new_n273), .o1(new_n277));
  aoai13aa1n12x5               g182(.a(new_n277), .b(new_n275), .c(new_n176), .d(new_n181), .o1(new_n278));
  xorb03aa1n02x5               g183(.a(new_n278), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  xorc02aa1n12x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnrc02aa1n12x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  aoai13aa1n03x5               g187(.a(new_n282), .b(new_n280), .c(new_n278), .d(new_n281), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n274), .b(new_n194), .c(new_n193), .d(new_n175), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n281), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n282), .b(new_n280), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n285), .c(new_n284), .d(new_n277), .o1(new_n287));
  nanp02aa1n03x5               g192(.a(new_n283), .b(new_n287), .o1(\s[28] ));
  norb02aa1n03x5               g193(.a(new_n281), .b(new_n282), .out0(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  xorc02aa1n12x5               g195(.a(\a[29] ), .b(\b[28] ), .out0(new_n291));
  inv000aa1d42x5               g196(.a(new_n291), .o1(new_n292));
  aoi012aa1n03x5               g197(.a(new_n286), .b(\a[28] ), .c(\b[27] ), .o1(new_n293));
  norp02aa1n02x5               g198(.a(new_n293), .b(new_n292), .o1(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n284), .d(new_n277), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n292), .b(new_n293), .c(new_n278), .d(new_n289), .o1(new_n296));
  nanp02aa1n03x5               g201(.a(new_n296), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g203(.a(new_n282), .b(new_n281), .c(new_n291), .out0(new_n299));
  nanp02aa1n03x5               g204(.a(new_n278), .b(new_n299), .o1(new_n300));
  norp02aa1n02x5               g205(.a(\b[28] ), .b(\a[29] ), .o1(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(new_n293), .c(new_n291), .o1(new_n302));
  xorc02aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .out0(new_n303));
  inv000aa1n02x5               g208(.a(new_n299), .o1(new_n304));
  oai012aa1n02x5               g209(.a(new_n303), .b(\b[28] ), .c(\a[29] ), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n305), .b(new_n293), .c(new_n291), .o1(new_n306));
  aoai13aa1n02x5               g211(.a(new_n306), .b(new_n304), .c(new_n284), .d(new_n277), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n303), .c(new_n300), .d(new_n302), .o1(\s[30] ));
  nano23aa1n06x5               g213(.a(new_n292), .b(new_n282), .c(new_n303), .d(new_n281), .out0(new_n309));
  inv000aa1d42x5               g214(.a(new_n309), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  aoi012aa1n02x5               g216(.a(new_n306), .b(\a[30] ), .c(\b[29] ), .o1(new_n312));
  norp02aa1n02x5               g217(.a(new_n312), .b(new_n311), .o1(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n310), .c(new_n284), .d(new_n277), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n311), .b(new_n312), .c(new_n278), .d(new_n309), .o1(new_n315));
  nanp02aa1n03x5               g220(.a(new_n315), .b(new_n314), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  orn002aa1n02x5               g222(.a(new_n104), .b(new_n102), .o(new_n318));
  xobna2aa1n03x5               g223(.a(new_n103), .b(new_n318), .c(new_n106), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g225(.a(new_n111), .o1(new_n321));
  norb02aa1n02x5               g226(.a(new_n114), .b(new_n113), .out0(new_n322));
  nanp03aa1n02x5               g227(.a(new_n108), .b(new_n321), .c(new_n112), .o1(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n322), .b(new_n323), .c(new_n321), .out0(\s[6] ));
  aoai13aa1n02x5               g229(.a(new_n117), .b(new_n115), .c(new_n187), .d(new_n107), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanp02aa1n02x5               g231(.a(new_n325), .b(new_n189), .o1(new_n327));
  xnbna2aa1n03x5               g232(.a(new_n188), .b(new_n327), .c(new_n119), .out0(\s[8] ));
  xorb03aa1n02x5               g233(.a(new_n128), .b(\b[8] ), .c(new_n97), .out0(\s[9] ));
endmodule


