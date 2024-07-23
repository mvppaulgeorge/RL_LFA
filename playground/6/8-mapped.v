// Benchmark "adder" written by ABC on Wed Jul 17 15:02:11 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n325, new_n327,
    new_n328, new_n329, new_n330, new_n331, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d24x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d06x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand02aa1d06x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1d18x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  xnrc02aa1n12x5               g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  nor042aa1d18x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanb02aa1n12x5               g010(.a(new_n104), .b(new_n105), .out0(new_n106));
  inv030aa1n03x5               g011(.a(new_n104), .o1(new_n107));
  oao003aa1n09x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .carry(new_n108));
  oai013aa1d12x5               g013(.a(new_n108), .b(new_n103), .c(new_n102), .d(new_n106), .o1(new_n109));
  nor022aa1n08x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n06x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1d18x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  tech160nm_fixnrc02aa1n02p5x5 g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  xnrc02aa1n12x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nor042aa1d18x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  inv040aa1n03x5               g023(.a(new_n118), .o1(new_n119));
  oaoi03aa1n03x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n121));
  oaib12aa1n06x5               g026(.a(new_n121), .b(new_n114), .c(new_n120), .out0(new_n122));
  nand42aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n06x5               g028(.a(new_n123), .b(new_n97), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n03x5               g032(.a(new_n109), .b(new_n117), .o1(new_n128));
  nona22aa1n02x4               g033(.a(new_n111), .b(new_n112), .c(new_n110), .out0(new_n129));
  aboi22aa1n06x5               g034(.a(new_n114), .b(new_n120), .c(new_n129), .d(new_n111), .out0(new_n130));
  and002aa1n02x5               g035(.a(new_n126), .b(new_n124), .o(new_n131));
  inv000aa1n02x5               g036(.a(new_n131), .o1(new_n132));
  nor042aa1n03x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand42aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  oaih12aa1n12x5               g039(.a(new_n134), .b(new_n133), .c(new_n97), .o1(new_n135));
  aoai13aa1n03x5               g040(.a(new_n135), .b(new_n132), .c(new_n128), .d(new_n130), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  nand02aa1n08x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n138), .out0(new_n141));
  nanp02aa1n03x5               g046(.a(new_n136), .b(new_n141), .o1(new_n142));
  nor002aa1d32x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1d10x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n142), .c(new_n139), .out0(\s[12] ));
  nona23aa1d18x5               g051(.a(new_n144), .b(new_n140), .c(new_n138), .d(new_n143), .out0(new_n147));
  nano22aa1n02x5               g052(.a(new_n147), .b(new_n126), .c(new_n124), .out0(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n149));
  oaih12aa1n12x5               g054(.a(new_n144), .b(new_n143), .c(new_n138), .o1(new_n150));
  oai012aa1d24x5               g055(.a(new_n150), .b(new_n147), .c(new_n135), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nor002aa1d32x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand02aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  norb02aa1n03x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n149), .c(new_n152), .out0(\s[13] ));
  aobi12aa1n02x5               g061(.a(new_n155), .b(new_n149), .c(new_n152), .out0(new_n157));
  nor002aa1n16x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand42aa1n08x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  oabi12aa1n02x5               g065(.a(new_n160), .b(new_n157), .c(new_n153), .out0(new_n161));
  norb03aa1n02x5               g066(.a(new_n159), .b(new_n153), .c(new_n158), .out0(new_n162));
  oaib12aa1n02x5               g067(.a(new_n161), .b(new_n157), .c(new_n162), .out0(\s[14] ));
  oaih12aa1n02x5               g068(.a(new_n159), .b(new_n158), .c(new_n153), .o1(new_n164));
  nona23aa1n09x5               g069(.a(new_n159), .b(new_n154), .c(new_n153), .d(new_n158), .out0(new_n165));
  aoai13aa1n04x5               g070(.a(new_n164), .b(new_n165), .c(new_n149), .d(new_n152), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n04x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanb02aa1n03x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  norb02aa1n02x5               g075(.a(new_n166), .b(new_n170), .out0(new_n171));
  nor042aa1n06x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand42aa1n04x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanb02aa1n03x5               g078(.a(new_n172), .b(new_n173), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n175));
  nona22aa1n02x4               g080(.a(new_n173), .b(new_n172), .c(new_n168), .out0(new_n176));
  oai012aa1n02x5               g081(.a(new_n175), .b(new_n171), .c(new_n176), .o1(\s[16] ));
  nano23aa1n03x7               g082(.a(new_n138), .b(new_n143), .c(new_n144), .d(new_n140), .out0(new_n178));
  nano23aa1n03x5               g083(.a(new_n168), .b(new_n172), .c(new_n173), .d(new_n169), .out0(new_n179));
  nand03aa1n02x5               g084(.a(new_n179), .b(new_n155), .c(new_n160), .o1(new_n180));
  nano32aa1n03x7               g085(.a(new_n180), .b(new_n178), .c(new_n126), .d(new_n124), .out0(new_n181));
  aoai13aa1n12x5               g086(.a(new_n181), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n182));
  nor043aa1n03x5               g087(.a(new_n165), .b(new_n174), .c(new_n170), .o1(new_n183));
  oai012aa1n02x5               g088(.a(new_n173), .b(new_n172), .c(new_n168), .o1(new_n184));
  oai013aa1n03x4               g089(.a(new_n184), .b(new_n164), .c(new_n170), .d(new_n174), .o1(new_n185));
  aoi012aa1n12x5               g090(.a(new_n185), .b(new_n151), .c(new_n183), .o1(new_n186));
  tech160nm_fixorc02aa1n04x5   g091(.a(\a[17] ), .b(\b[16] ), .out0(new_n187));
  xnbna2aa1n03x5               g092(.a(new_n187), .b(new_n182), .c(new_n186), .out0(\s[17] ));
  inv040aa1d32x5               g093(.a(\a[17] ), .o1(new_n189));
  inv040aa1d32x5               g094(.a(\b[16] ), .o1(new_n190));
  nanp02aa1n24x5               g095(.a(new_n190), .b(new_n189), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(new_n148), .b(new_n183), .o1(new_n192));
  aoai13aa1n09x5               g097(.a(new_n186), .b(new_n192), .c(new_n128), .d(new_n130), .o1(new_n193));
  nand22aa1n03x5               g098(.a(new_n193), .b(new_n187), .o1(new_n194));
  norp02aa1n02x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  xobna2aa1n03x5               g102(.a(new_n197), .b(new_n194), .c(new_n191), .out0(\s[18] ));
  inv040aa1n08x5               g103(.a(\a[18] ), .o1(new_n199));
  xroi22aa1d06x4               g104(.a(new_n189), .b(\b[16] ), .c(new_n199), .d(\b[17] ), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  oaoi03aa1n12x5               g106(.a(\a[18] ), .b(\b[17] ), .c(new_n191), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n201), .c(new_n182), .d(new_n186), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand42aa1d28x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nor042aa1n09x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand42aa1d28x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  aoai13aa1n03x5               g116(.a(new_n211), .b(new_n207), .c(new_n204), .d(new_n208), .o1(new_n212));
  nanp02aa1n03x5               g117(.a(new_n193), .b(new_n200), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n207), .b(new_n208), .out0(new_n214));
  norb03aa1n02x5               g119(.a(new_n210), .b(new_n207), .c(new_n209), .out0(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n214), .c(new_n213), .d(new_n203), .o1(new_n216));
  nanp02aa1n03x5               g121(.a(new_n212), .b(new_n216), .o1(\s[20] ));
  nano23aa1d15x5               g122(.a(new_n207), .b(new_n209), .c(new_n210), .d(new_n208), .out0(new_n218));
  nanb03aa1n06x5               g123(.a(new_n197), .b(new_n218), .c(new_n187), .out0(new_n219));
  nand22aa1n09x5               g124(.a(new_n218), .b(new_n202), .o1(new_n220));
  oai012aa1d24x5               g125(.a(new_n210), .b(new_n209), .c(new_n207), .o1(new_n221));
  nand22aa1n09x5               g126(.a(new_n220), .b(new_n221), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n219), .c(new_n182), .d(new_n186), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n20x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  nand42aa1d28x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(new_n228));
  nor002aa1d32x5               g133(.a(\b[21] ), .b(\a[22] ), .o1(new_n229));
  nand02aa1n20x5               g134(.a(\b[21] ), .b(\a[22] ), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n226), .c(new_n224), .d(new_n228), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(new_n224), .b(new_n228), .o1(new_n234));
  norb03aa1n02x5               g139(.a(new_n230), .b(new_n226), .c(new_n229), .out0(new_n235));
  nanp02aa1n02x5               g140(.a(new_n234), .b(new_n235), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n233), .b(new_n236), .o1(\s[22] ));
  nano23aa1d15x5               g142(.a(new_n226), .b(new_n229), .c(new_n230), .d(new_n227), .out0(new_n238));
  nand03aa1n02x5               g143(.a(new_n200), .b(new_n218), .c(new_n238), .o1(new_n239));
  and002aa1n02x5               g144(.a(\b[21] ), .b(\a[22] ), .o(new_n240));
  oab012aa1n02x4               g145(.a(new_n240), .b(new_n226), .c(new_n229), .out0(new_n241));
  aoi012aa1n02x7               g146(.a(new_n241), .b(new_n222), .c(new_n238), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n239), .c(new_n182), .d(new_n186), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  inv000aa1d42x5               g149(.a(new_n218), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n238), .o1(new_n246));
  nona32aa1n02x4               g151(.a(new_n193), .b(new_n246), .c(new_n245), .d(new_n201), .out0(new_n247));
  nor002aa1d32x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  tech160nm_finand02aa1n03p5x5 g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  aoi012aa1n03x5               g156(.a(new_n251), .b(new_n247), .c(new_n242), .o1(new_n252));
  nor022aa1n08x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  nand42aa1n08x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  nanb02aa1n02x5               g159(.a(new_n253), .b(new_n254), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n248), .c(new_n243), .d(new_n250), .o1(new_n256));
  nona22aa1n03x5               g161(.a(new_n254), .b(new_n253), .c(new_n248), .out0(new_n257));
  oai012aa1n03x5               g162(.a(new_n256), .b(new_n252), .c(new_n257), .o1(\s[24] ));
  nano23aa1n09x5               g163(.a(new_n248), .b(new_n253), .c(new_n254), .d(new_n249), .out0(new_n259));
  nona23aa1n06x5               g164(.a(new_n200), .b(new_n259), .c(new_n246), .d(new_n245), .out0(new_n260));
  nand22aa1n02x5               g165(.a(new_n259), .b(new_n238), .o1(new_n261));
  aoi022aa1n06x5               g166(.a(new_n259), .b(new_n241), .c(new_n254), .d(new_n257), .o1(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n261), .c(new_n220), .d(new_n221), .o1(new_n263));
  inv000aa1n02x5               g168(.a(new_n263), .o1(new_n264));
  aoai13aa1n02x7               g169(.a(new_n264), .b(new_n260), .c(new_n182), .d(new_n186), .o1(new_n265));
  xorb03aa1n02x5               g170(.a(new_n265), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nona32aa1n03x5               g171(.a(new_n193), .b(new_n261), .c(new_n245), .d(new_n201), .out0(new_n267));
  xnrc02aa1n12x5               g172(.a(\b[24] ), .b(\a[25] ), .out0(new_n268));
  aoi012aa1n03x5               g173(.a(new_n268), .b(new_n267), .c(new_n264), .o1(new_n269));
  norp02aa1n02x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n268), .o1(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[25] ), .b(\a[26] ), .out0(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n270), .c(new_n265), .d(new_n271), .o1(new_n273));
  oabi12aa1n02x5               g178(.a(new_n272), .b(\a[25] ), .c(\b[24] ), .out0(new_n274));
  oai012aa1n03x5               g179(.a(new_n273), .b(new_n269), .c(new_n274), .o1(\s[26] ));
  nor042aa1n02x5               g180(.a(new_n272), .b(new_n268), .o1(new_n276));
  nano32aa1n03x7               g181(.a(new_n219), .b(new_n276), .c(new_n238), .d(new_n259), .out0(new_n277));
  inv020aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[25] ), .b(\a[26] ), .o1(new_n279));
  aoi022aa1n06x5               g184(.a(new_n263), .b(new_n276), .c(new_n279), .d(new_n274), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n280), .b(new_n278), .c(new_n182), .d(new_n186), .o1(new_n281));
  xorb03aa1n03x5               g186(.a(new_n281), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[27] ), .b(\b[26] ), .out0(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n283), .c(new_n281), .d(new_n284), .o1(new_n286));
  nand22aa1n03x5               g191(.a(new_n263), .b(new_n276), .o1(new_n287));
  oai012aa1n02x5               g192(.a(new_n279), .b(new_n272), .c(new_n270), .o1(new_n288));
  nand02aa1d04x5               g193(.a(new_n287), .b(new_n288), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n284), .b(new_n289), .c(new_n193), .d(new_n277), .o1(new_n290));
  nona22aa1n03x5               g195(.a(new_n290), .b(new_n285), .c(new_n283), .out0(new_n291));
  nanp02aa1n03x5               g196(.a(new_n286), .b(new_n291), .o1(\s[28] ));
  tech160nm_fixorc02aa1n03p5x5 g197(.a(\a[29] ), .b(\b[28] ), .out0(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  norb02aa1n02x5               g199(.a(new_n284), .b(new_n285), .out0(new_n295));
  inv000aa1d42x5               g200(.a(\a[28] ), .o1(new_n296));
  inv000aa1d42x5               g201(.a(\b[27] ), .o1(new_n297));
  oaoi03aa1n06x5               g202(.a(new_n296), .b(new_n297), .c(new_n283), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n294), .b(new_n299), .c(new_n281), .d(new_n295), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n295), .b(new_n289), .c(new_n193), .d(new_n277), .o1(new_n301));
  nona22aa1n03x5               g206(.a(new_n301), .b(new_n299), .c(new_n294), .out0(new_n302));
  nanp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[29] ));
  xorb03aa1n02x5               g208(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g209(.a(new_n285), .b(new_n284), .c(new_n293), .out0(new_n305));
  oaoi03aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .o1(new_n306));
  tech160nm_fixorc02aa1n03p5x5 g211(.a(\a[30] ), .b(\b[29] ), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n307), .o1(new_n308));
  aoai13aa1n02x5               g213(.a(new_n308), .b(new_n306), .c(new_n281), .d(new_n305), .o1(new_n309));
  aoai13aa1n02x7               g214(.a(new_n305), .b(new_n289), .c(new_n193), .d(new_n277), .o1(new_n310));
  nona22aa1n02x4               g215(.a(new_n310), .b(new_n306), .c(new_n308), .out0(new_n311));
  nanp02aa1n03x5               g216(.a(new_n309), .b(new_n311), .o1(\s[30] ));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  nano23aa1n02x4               g218(.a(new_n308), .b(new_n285), .c(new_n284), .d(new_n293), .out0(new_n314));
  and002aa1n02x5               g219(.a(\b[29] ), .b(\a[30] ), .o(new_n315));
  oab012aa1n02x4               g220(.a(new_n315), .b(new_n306), .c(new_n308), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n313), .b(new_n316), .c(new_n281), .d(new_n314), .o1(new_n317));
  aoai13aa1n03x5               g222(.a(new_n314), .b(new_n289), .c(new_n193), .d(new_n277), .o1(new_n318));
  nona22aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n313), .out0(new_n319));
  nanp02aa1n03x5               g224(.a(new_n317), .b(new_n319), .o1(\s[31] ));
  xnbna2aa1n03x5               g225(.a(new_n102), .b(new_n105), .c(new_n107), .out0(\s[3] ));
  orn002aa1n02x5               g226(.a(new_n102), .b(new_n106), .o(new_n322));
  xobna2aa1n03x5               g227(.a(new_n103), .b(new_n322), .c(new_n107), .out0(\s[4] ));
  xorb03aa1n02x5               g228(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n06x5               g229(.a(new_n115), .b(new_n109), .out0(new_n325));
  xobna2aa1n03x5               g230(.a(new_n116), .b(new_n325), .c(new_n119), .out0(\s[6] ));
  and002aa1n02x5               g231(.a(\b[5] ), .b(\a[6] ), .o(new_n327));
  nona22aa1n02x4               g232(.a(new_n325), .b(new_n116), .c(new_n118), .out0(new_n328));
  nona23aa1n03x5               g233(.a(new_n328), .b(new_n113), .c(new_n112), .d(new_n327), .out0(new_n329));
  inv000aa1d42x5               g234(.a(new_n112), .o1(new_n330));
  aboi22aa1n03x5               g235(.a(new_n327), .b(new_n328), .c(new_n330), .d(new_n113), .out0(new_n331));
  norb02aa1n02x5               g236(.a(new_n329), .b(new_n331), .out0(\s[7] ));
  nanb02aa1n02x5               g237(.a(new_n110), .b(new_n111), .out0(new_n333));
  xobna2aa1n03x5               g238(.a(new_n333), .b(new_n329), .c(new_n330), .out0(\s[8] ));
  xnbna2aa1n03x5               g239(.a(new_n124), .b(new_n128), .c(new_n130), .out0(\s[9] ));
endmodule


