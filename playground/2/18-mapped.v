// Benchmark "adder" written by ABC on Wed Jul 17 13:04:56 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n312, new_n313,
    new_n314, new_n316, new_n317, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor002aa1d32x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nand02aa1d06x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nor042aa1n04x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand22aa1n04x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor042aa1n04x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n06x5               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  tech160nm_fixorc02aa1n02p5x5 g009(.a(\a[6] ), .b(\b[5] ), .out0(new_n105));
  tech160nm_fixorc02aa1n04x5   g010(.a(\a[5] ), .b(\b[4] ), .out0(new_n106));
  nanp03aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nand02aa1d28x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nano22aa1d15x5               g016(.a(new_n110), .b(new_n109), .c(new_n111), .out0(new_n112));
  nand22aa1n03x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nor042aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nona22aa1n02x4               g019(.a(new_n109), .b(new_n114), .c(new_n113), .out0(new_n115));
  oab012aa1d15x5               g020(.a(new_n110), .b(\a[4] ), .c(\b[3] ), .out0(new_n116));
  inv000aa1d42x5               g021(.a(new_n116), .o1(new_n117));
  aoai13aa1n06x5               g022(.a(new_n108), .b(new_n117), .c(new_n112), .d(new_n115), .o1(new_n118));
  nor042aa1n09x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(new_n119), .o1(new_n120));
  oaoi03aa1n12x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  oaih12aa1n02x5               g026(.a(new_n101), .b(new_n102), .c(new_n100), .o1(new_n122));
  aobi12aa1n12x5               g027(.a(new_n122), .b(new_n104), .c(new_n121), .out0(new_n123));
  oai012aa1n09x5               g028(.a(new_n123), .b(new_n118), .c(new_n107), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n97), .b(new_n98), .c(new_n124), .d(new_n99), .o1(new_n125));
  nona23aa1n02x4               g030(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n126));
  nano22aa1n03x7               g031(.a(new_n126), .b(new_n105), .c(new_n106), .out0(new_n127));
  norb02aa1n15x5               g032(.a(new_n111), .b(new_n110), .out0(new_n128));
  oai112aa1n04x5               g033(.a(new_n128), .b(new_n109), .c(new_n113), .d(new_n114), .o1(new_n129));
  aoi022aa1n12x5               g034(.a(new_n129), .b(new_n116), .c(\b[3] ), .d(\a[4] ), .o1(new_n130));
  inv000aa1n06x5               g035(.a(new_n123), .o1(new_n131));
  aoai13aa1n02x5               g036(.a(new_n99), .b(new_n131), .c(new_n130), .d(new_n127), .o1(new_n132));
  nona22aa1n02x4               g037(.a(new_n132), .b(new_n98), .c(new_n97), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n125), .b(new_n133), .o1(\s[10] ));
  nanp02aa1n02x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nand42aa1d28x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor002aa1n16x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  xobna2aa1n03x5               g043(.a(new_n138), .b(new_n133), .c(new_n135), .out0(\s[11] ));
  aoi013aa1n02x4               g044(.a(new_n137), .b(new_n133), .c(new_n135), .d(new_n138), .o1(new_n140));
  nor042aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1d28x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  nona22aa1n09x5               g048(.a(new_n142), .b(new_n141), .c(new_n137), .out0(new_n144));
  aoi013aa1n02x4               g049(.a(new_n144), .b(new_n133), .c(new_n138), .d(new_n135), .o1(new_n145));
  oabi12aa1n02x7               g050(.a(new_n145), .b(new_n140), .c(new_n143), .out0(\s[12] ));
  nano23aa1d15x5               g051(.a(new_n141), .b(new_n137), .c(new_n142), .d(new_n136), .out0(new_n147));
  nanb02aa1n12x5               g052(.a(new_n98), .b(new_n99), .out0(new_n148));
  nona22aa1d30x5               g053(.a(new_n147), .b(new_n97), .c(new_n148), .out0(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n131), .c(new_n130), .d(new_n127), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n98), .o1(new_n152));
  oaoi03aa1n02x5               g057(.a(\a[10] ), .b(\b[9] ), .c(new_n152), .o1(new_n153));
  aoi022aa1n06x5               g058(.a(new_n147), .b(new_n153), .c(new_n144), .d(new_n142), .o1(new_n154));
  xnrc02aa1n12x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  xobna2aa1n03x5               g060(.a(new_n155), .b(new_n151), .c(new_n154), .out0(\s[13] ));
  orn002aa1n02x5               g061(.a(\a[13] ), .b(\b[12] ), .o(new_n157));
  tech160nm_fiao0012aa1n02p5x5 g062(.a(new_n155), .b(new_n151), .c(new_n154), .o(new_n158));
  xnrc02aa1n12x5               g063(.a(\b[13] ), .b(\a[14] ), .out0(new_n159));
  xobna2aa1n03x5               g064(.a(new_n159), .b(new_n158), .c(new_n157), .out0(\s[14] ));
  nor042aa1n12x5               g065(.a(new_n159), .b(new_n155), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  nor002aa1n02x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  norp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  tech160nm_fioai012aa1n03p5x5 g070(.a(new_n165), .b(new_n164), .c(new_n163), .o1(new_n166));
  aoai13aa1n03x5               g071(.a(new_n166), .b(new_n162), .c(new_n151), .d(new_n154), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  xnrc02aa1n12x5               g073(.a(\b[14] ), .b(\a[15] ), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  norp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  inv040aa1d32x5               g076(.a(\a[16] ), .o1(new_n172));
  inv000aa1d42x5               g077(.a(\b[15] ), .o1(new_n173));
  nand02aa1n02x5               g078(.a(new_n173), .b(new_n172), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand02aa1d04x5               g080(.a(new_n174), .b(new_n175), .o1(new_n176));
  aoai13aa1n03x5               g081(.a(new_n176), .b(new_n171), .c(new_n167), .d(new_n170), .o1(new_n177));
  oai112aa1n02x5               g082(.a(new_n174), .b(new_n175), .c(\b[14] ), .d(\a[15] ), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n177), .b(new_n178), .c(new_n170), .d(new_n167), .o1(\s[16] ));
  nor042aa1n06x5               g084(.a(new_n169), .b(new_n176), .o1(new_n180));
  nano22aa1d15x5               g085(.a(new_n149), .b(new_n161), .c(new_n180), .out0(new_n181));
  aoai13aa1n12x5               g086(.a(new_n181), .b(new_n131), .c(new_n130), .d(new_n127), .o1(new_n182));
  nand42aa1n03x5               g087(.a(new_n161), .b(new_n180), .o1(new_n183));
  oaoi03aa1n02x5               g088(.a(new_n172), .b(new_n173), .c(new_n171), .o1(new_n184));
  oai013aa1n06x5               g089(.a(new_n184), .b(new_n169), .c(new_n166), .d(new_n176), .o1(new_n185));
  oab012aa1n09x5               g090(.a(new_n185), .b(new_n154), .c(new_n183), .out0(new_n186));
  xorc02aa1n02x5               g091(.a(\a[17] ), .b(\b[16] ), .out0(new_n187));
  xnbna2aa1n03x5               g092(.a(new_n187), .b(new_n182), .c(new_n186), .out0(\s[17] ));
  inv000aa1d42x5               g093(.a(\a[17] ), .o1(new_n189));
  nanb02aa1n02x5               g094(.a(\b[16] ), .b(new_n189), .out0(new_n190));
  oabi12aa1n06x5               g095(.a(new_n185), .b(new_n154), .c(new_n183), .out0(new_n191));
  aoai13aa1n02x5               g096(.a(new_n187), .b(new_n191), .c(new_n124), .d(new_n181), .o1(new_n192));
  xnrc02aa1n02x5               g097(.a(\b[17] ), .b(\a[18] ), .out0(new_n193));
  xobna2aa1n03x5               g098(.a(new_n193), .b(new_n192), .c(new_n190), .out0(\s[18] ));
  inv000aa1d42x5               g099(.a(\a[18] ), .o1(new_n195));
  xroi22aa1d04x5               g100(.a(new_n189), .b(\b[16] ), .c(new_n195), .d(\b[17] ), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n191), .c(new_n124), .d(new_n181), .o1(new_n197));
  oaih22aa1n04x5               g102(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n198));
  oaib12aa1n09x5               g103(.a(new_n198), .b(new_n195), .c(\b[17] ), .out0(new_n199));
  nor022aa1n16x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand22aa1n04x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n197), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n06x5               g109(.a(new_n202), .b(new_n197), .c(new_n199), .out0(new_n205));
  nor002aa1n16x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand22aa1n09x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(new_n208));
  tech160nm_fioai012aa1n05x5   g113(.a(new_n208), .b(new_n205), .c(new_n200), .o1(new_n209));
  norb03aa1n02x5               g114(.a(new_n207), .b(new_n200), .c(new_n206), .out0(new_n210));
  oaib12aa1n02x5               g115(.a(new_n209), .b(new_n205), .c(new_n210), .out0(\s[20] ));
  nano23aa1n06x5               g116(.a(new_n200), .b(new_n206), .c(new_n207), .d(new_n201), .out0(new_n212));
  nanb03aa1n06x5               g117(.a(new_n193), .b(new_n212), .c(new_n187), .out0(new_n213));
  nona23aa1d18x5               g118(.a(new_n207), .b(new_n201), .c(new_n200), .d(new_n206), .out0(new_n214));
  oai012aa1d24x5               g119(.a(new_n207), .b(new_n206), .c(new_n200), .o1(new_n215));
  oai012aa1d24x5               g120(.a(new_n215), .b(new_n214), .c(new_n199), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n06x5               g122(.a(new_n217), .b(new_n213), .c(new_n182), .d(new_n186), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xnrc02aa1n02x5               g126(.a(\b[21] ), .b(\a[22] ), .out0(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n221), .c(new_n218), .d(new_n220), .o1(new_n223));
  oabi12aa1n02x5               g128(.a(new_n222), .b(\a[21] ), .c(\b[20] ), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n224), .c(new_n220), .d(new_n218), .o1(\s[22] ));
  inv000aa1d42x5               g130(.a(\a[21] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\a[22] ), .o1(new_n227));
  xroi22aa1d06x4               g132(.a(new_n226), .b(\b[20] ), .c(new_n227), .d(\b[21] ), .out0(new_n228));
  nand23aa1n04x5               g133(.a(new_n228), .b(new_n196), .c(new_n212), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\b[21] ), .o1(new_n230));
  oao003aa1n12x5               g135(.a(new_n227), .b(new_n230), .c(new_n221), .carry(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n216), .c(new_n228), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n229), .c(new_n182), .d(new_n186), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  xorc02aa1n02x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xnrc02aa1n12x5               g141(.a(\b[23] ), .b(\a[24] ), .out0(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n236), .c(new_n233), .d(new_n235), .o1(new_n238));
  oabi12aa1n02x5               g143(.a(new_n237), .b(\a[23] ), .c(\b[22] ), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n238), .b(new_n239), .c(new_n235), .d(new_n233), .o1(\s[24] ));
  norb02aa1n06x5               g145(.a(new_n235), .b(new_n237), .out0(new_n241));
  nanb03aa1n02x5               g146(.a(new_n213), .b(new_n241), .c(new_n228), .out0(new_n242));
  oaoi03aa1n02x5               g147(.a(\a[18] ), .b(\b[17] ), .c(new_n190), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n215), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n228), .b(new_n244), .c(new_n212), .d(new_n243), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n231), .o1(new_n246));
  inv000aa1n02x5               g151(.a(new_n241), .o1(new_n247));
  aob012aa1n02x5               g152(.a(new_n239), .b(\b[23] ), .c(\a[24] ), .out0(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n247), .c(new_n245), .d(new_n246), .o1(new_n249));
  inv000aa1n02x5               g154(.a(new_n249), .o1(new_n250));
  aoai13aa1n04x5               g155(.a(new_n250), .b(new_n242), .c(new_n182), .d(new_n186), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  tech160nm_fixorc02aa1n02p5x5 g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  tech160nm_fixnrc02aa1n02p5x5 g159(.a(\b[25] ), .b(\a[26] ), .out0(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n256));
  norp02aa1n02x5               g161(.a(new_n255), .b(new_n253), .o1(new_n257));
  aob012aa1n02x5               g162(.a(new_n257), .b(new_n251), .c(new_n254), .out0(new_n258));
  nanp02aa1n03x5               g163(.a(new_n256), .b(new_n258), .o1(\s[26] ));
  norb02aa1n06x5               g164(.a(new_n254), .b(new_n255), .out0(new_n260));
  nano22aa1d15x5               g165(.a(new_n229), .b(new_n241), .c(new_n260), .out0(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n191), .c(new_n124), .d(new_n181), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(new_n249), .b(new_n260), .o1(new_n263));
  tech160nm_fiao0012aa1n02p5x5 g168(.a(new_n257), .b(\a[26] ), .c(\b[25] ), .o(new_n264));
  nanp03aa1n06x5               g169(.a(new_n262), .b(new_n263), .c(new_n264), .o1(new_n265));
  xorb03aa1n02x5               g170(.a(new_n265), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  inv040aa1n02x5               g171(.a(new_n261), .o1(new_n267));
  aoi012aa1n12x5               g172(.a(new_n267), .b(new_n182), .c(new_n186), .o1(new_n268));
  aoai13aa1n04x5               g173(.a(new_n241), .b(new_n231), .c(new_n216), .d(new_n228), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n260), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n264), .b(new_n270), .c(new_n269), .d(new_n248), .o1(new_n271));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  xorc02aa1n02x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  oaoi13aa1n03x5               g178(.a(new_n272), .b(new_n273), .c(new_n271), .d(new_n268), .o1(new_n274));
  xorc02aa1n02x5               g179(.a(\a[28] ), .b(\b[27] ), .out0(new_n275));
  oaih12aa1n02x5               g180(.a(new_n273), .b(new_n271), .c(new_n268), .o1(new_n276));
  oai112aa1n02x7               g181(.a(new_n276), .b(new_n275), .c(\b[26] ), .d(\a[27] ), .o1(new_n277));
  oaih12aa1n02x5               g182(.a(new_n277), .b(new_n274), .c(new_n275), .o1(\s[28] ));
  xorc02aa1n12x5               g183(.a(\a[29] ), .b(\b[28] ), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  and002aa1n02x5               g185(.a(new_n275), .b(new_n273), .o(new_n281));
  orn002aa1n02x5               g186(.a(\a[27] ), .b(\b[26] ), .o(new_n282));
  oaoi03aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .o1(new_n283));
  aoai13aa1n02x5               g188(.a(new_n280), .b(new_n283), .c(new_n265), .d(new_n281), .o1(new_n284));
  oaih12aa1n02x5               g189(.a(new_n281), .b(new_n271), .c(new_n268), .o1(new_n285));
  nona22aa1n03x5               g190(.a(new_n285), .b(new_n283), .c(new_n280), .out0(new_n286));
  nanp02aa1n02x5               g191(.a(new_n284), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g193(.a(new_n280), .b(new_n273), .c(new_n275), .out0(new_n289));
  inv000aa1d42x5               g194(.a(\b[28] ), .o1(new_n290));
  oaib12aa1n02x5               g195(.a(new_n283), .b(new_n290), .c(\a[29] ), .out0(new_n291));
  oaib12aa1n02x5               g196(.a(new_n291), .b(\a[29] ), .c(new_n290), .out0(new_n292));
  oaoi13aa1n03x5               g197(.a(new_n292), .b(new_n289), .c(new_n271), .d(new_n268), .o1(new_n293));
  xorc02aa1n02x5               g198(.a(\a[30] ), .b(\b[29] ), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n289), .b(new_n271), .c(new_n268), .o1(new_n295));
  norb02aa1n02x5               g200(.a(new_n294), .b(new_n292), .out0(new_n296));
  tech160nm_finand02aa1n05x5   g201(.a(new_n295), .b(new_n296), .o1(new_n297));
  oaih12aa1n02x5               g202(.a(new_n297), .b(new_n293), .c(new_n294), .o1(\s[30] ));
  nano32aa1n02x4               g203(.a(new_n280), .b(new_n294), .c(new_n273), .d(new_n275), .out0(new_n299));
  aoi012aa1n02x5               g204(.a(new_n296), .b(\a[30] ), .c(\b[29] ), .o1(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n300), .c(new_n265), .d(new_n299), .o1(new_n302));
  oaih12aa1n02x5               g207(.a(new_n299), .b(new_n271), .c(new_n268), .o1(new_n303));
  nona22aa1n03x5               g208(.a(new_n303), .b(new_n300), .c(new_n301), .out0(new_n304));
  nanp02aa1n03x5               g209(.a(new_n302), .b(new_n304), .o1(\s[31] ));
  xobna2aa1n03x5               g210(.a(new_n128), .b(new_n115), .c(new_n109), .out0(\s[3] ));
  aoi012aa1n02x5               g211(.a(new_n110), .b(new_n112), .c(new_n115), .o1(new_n307));
  xnrb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnrc02aa1n02x5               g213(.a(new_n118), .b(new_n106), .out0(\s[5] ));
  nanp02aa1n02x5               g214(.a(new_n130), .b(new_n106), .o1(new_n310));
  xnbna2aa1n03x5               g215(.a(new_n105), .b(new_n310), .c(new_n120), .out0(\s[6] ));
  nanb02aa1n02x5               g216(.a(new_n102), .b(new_n103), .out0(new_n312));
  nanp02aa1n02x5               g217(.a(\b[5] ), .b(\a[6] ), .o1(new_n313));
  nanp03aa1n02x5               g218(.a(new_n310), .b(new_n105), .c(new_n120), .o1(new_n314));
  xnbna2aa1n03x5               g219(.a(new_n312), .b(new_n314), .c(new_n313), .out0(\s[7] ));
  norb03aa1n02x5               g220(.a(new_n101), .b(new_n100), .c(new_n102), .out0(new_n316));
  nano22aa1n02x4               g221(.a(new_n312), .b(new_n314), .c(new_n313), .out0(new_n317));
  obai22aa1n02x7               g222(.a(new_n101), .b(new_n100), .c(new_n317), .d(new_n102), .out0(new_n318));
  oaib12aa1n02x5               g223(.a(new_n318), .b(new_n317), .c(new_n316), .out0(\s[8] ));
  xorb03aa1n02x5               g224(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


