// Benchmark "adder" written by ABC on Thu Jul 18 07:29:35 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n325, new_n326,
    new_n327, new_n329, new_n331;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n24x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n06x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  xnrc02aa1n12x5               g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  nor022aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanb02aa1n06x5               g010(.a(new_n104), .b(new_n105), .out0(new_n106));
  inv000aa1n02x5               g011(.a(new_n104), .o1(new_n107));
  oao003aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .carry(new_n108));
  oai013aa1d12x5               g013(.a(new_n108), .b(new_n103), .c(new_n102), .d(new_n106), .o1(new_n109));
  tech160nm_fixnrc02aa1n05x5   g014(.a(\b[7] ), .b(\a[8] ), .out0(new_n110));
  tech160nm_fixnrc02aa1n04x5   g015(.a(\b[6] ), .b(\a[7] ), .out0(new_n111));
  nor042aa1n04x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nor042aa1n03x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n114), .b(new_n113), .c(new_n115), .d(new_n112), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n116), .b(new_n111), .c(new_n110), .o1(new_n117));
  tech160nm_fioai012aa1n04x5   g022(.a(new_n114), .b(new_n115), .c(new_n112), .o1(new_n118));
  nor042aa1n06x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(new_n119), .o1(new_n120));
  oao003aa1n02x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .carry(new_n121));
  oai013aa1n03x5               g026(.a(new_n121), .b(new_n111), .c(new_n110), .d(new_n118), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n03x5               g028(.a(new_n123), .b(new_n97), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n125));
  xnrc02aa1n12x5               g030(.a(\b[9] ), .b(\a[10] ), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  nano22aa1n03x7               g033(.a(new_n126), .b(new_n98), .c(new_n123), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n130));
  nor002aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  oai012aa1n09x5               g037(.a(new_n132), .b(new_n131), .c(new_n97), .o1(new_n133));
  nand02aa1n03x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nor002aa1d32x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n130), .c(new_n133), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(new_n135), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(new_n130), .b(new_n133), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n139), .b(new_n136), .o1(new_n140));
  nor022aa1n08x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand22aa1n04x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n138), .out0(\s[12] ));
  nona23aa1n09x5               g049(.a(new_n134), .b(new_n142), .c(new_n141), .d(new_n135), .out0(new_n145));
  norb03aa1n02x5               g050(.a(new_n124), .b(new_n145), .c(new_n126), .out0(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n147));
  tech160nm_fioai012aa1n03p5x5 g052(.a(new_n142), .b(new_n141), .c(new_n135), .o1(new_n148));
  oai012aa1n18x5               g053(.a(new_n148), .b(new_n145), .c(new_n133), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  nand02aa1n03x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nor022aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  norb02aa1n02x7               g057(.a(new_n151), .b(new_n152), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n147), .c(new_n150), .out0(\s[13] ));
  aobi12aa1n02x5               g059(.a(new_n153), .b(new_n147), .c(new_n150), .out0(new_n155));
  norp02aa1n04x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  oabi12aa1n02x5               g063(.a(new_n158), .b(new_n155), .c(new_n152), .out0(new_n159));
  norb03aa1n02x5               g064(.a(new_n157), .b(new_n152), .c(new_n156), .out0(new_n160));
  oaib12aa1n02x5               g065(.a(new_n159), .b(new_n155), .c(new_n160), .out0(\s[14] ));
  oai012aa1n02x7               g066(.a(new_n157), .b(new_n156), .c(new_n152), .o1(new_n162));
  nona23aa1n06x5               g067(.a(new_n151), .b(new_n157), .c(new_n156), .d(new_n152), .out0(new_n163));
  aoai13aa1n03x5               g068(.a(new_n162), .b(new_n163), .c(new_n147), .d(new_n150), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nand42aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nor042aa1n06x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nanb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  norb02aa1n02x5               g073(.a(new_n164), .b(new_n168), .out0(new_n169));
  nor002aa1n03x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n03x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n167), .c(new_n164), .d(new_n166), .o1(new_n173));
  nona22aa1n02x4               g078(.a(new_n171), .b(new_n170), .c(new_n167), .out0(new_n174));
  oai012aa1n02x5               g079(.a(new_n173), .b(new_n169), .c(new_n174), .o1(\s[16] ));
  nano23aa1n02x5               g080(.a(new_n141), .b(new_n135), .c(new_n142), .d(new_n134), .out0(new_n176));
  nano23aa1n03x7               g081(.a(new_n170), .b(new_n167), .c(new_n171), .d(new_n166), .out0(new_n177));
  nand23aa1n03x5               g082(.a(new_n177), .b(new_n153), .c(new_n158), .o1(new_n178));
  nano32aa1n03x7               g083(.a(new_n178), .b(new_n176), .c(new_n127), .d(new_n124), .out0(new_n179));
  aoai13aa1n12x5               g084(.a(new_n179), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n180));
  nor043aa1n02x5               g085(.a(new_n163), .b(new_n172), .c(new_n168), .o1(new_n181));
  oai012aa1n02x5               g086(.a(new_n171), .b(new_n170), .c(new_n167), .o1(new_n182));
  oai013aa1n03x5               g087(.a(new_n182), .b(new_n162), .c(new_n168), .d(new_n172), .o1(new_n183));
  aoi012aa1d18x5               g088(.a(new_n183), .b(new_n149), .c(new_n181), .o1(new_n184));
  xorc02aa1n02x5               g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n180), .c(new_n184), .out0(\s[17] ));
  inv040aa1d32x5               g091(.a(\a[17] ), .o1(new_n187));
  inv040aa1d28x5               g092(.a(\b[16] ), .o1(new_n188));
  nand42aa1d28x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  nanp02aa1n03x5               g094(.a(new_n109), .b(new_n117), .o1(new_n190));
  norp03aa1n02x5               g095(.a(new_n110), .b(new_n111), .c(new_n118), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n121), .b(new_n191), .out0(new_n192));
  nona23aa1n03x5               g097(.a(new_n129), .b(new_n177), .c(new_n163), .d(new_n145), .out0(new_n193));
  aoai13aa1n09x5               g098(.a(new_n184), .b(new_n193), .c(new_n190), .d(new_n192), .o1(new_n194));
  nand22aa1n03x5               g099(.a(new_n194), .b(new_n185), .o1(new_n195));
  norp02aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  nanb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(new_n198));
  xobna2aa1n03x5               g103(.a(new_n198), .b(new_n195), .c(new_n189), .out0(\s[18] ));
  inv040aa1d32x5               g104(.a(\a[18] ), .o1(new_n200));
  xroi22aa1d04x5               g105(.a(new_n187), .b(\b[16] ), .c(new_n200), .d(\b[17] ), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  oaoi03aa1n12x5               g107(.a(\a[18] ), .b(\b[17] ), .c(new_n189), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n202), .c(new_n180), .d(new_n184), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n20x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nor042aa1n09x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nor042aa1n09x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand42aa1n20x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n209), .c(new_n205), .d(new_n208), .o1(new_n213));
  tech160nm_finand02aa1n05x5   g118(.a(new_n194), .b(new_n201), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(new_n209), .b(new_n208), .out0(new_n215));
  norb03aa1n02x5               g120(.a(new_n211), .b(new_n209), .c(new_n210), .out0(new_n216));
  aoai13aa1n03x5               g121(.a(new_n216), .b(new_n215), .c(new_n214), .d(new_n204), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(new_n213), .b(new_n217), .o1(\s[20] ));
  nano23aa1d15x5               g123(.a(new_n210), .b(new_n209), .c(new_n211), .d(new_n208), .out0(new_n219));
  nanb03aa1n06x5               g124(.a(new_n198), .b(new_n219), .c(new_n185), .out0(new_n220));
  nand02aa1d06x5               g125(.a(new_n219), .b(new_n203), .o1(new_n221));
  oaih12aa1n12x5               g126(.a(new_n211), .b(new_n210), .c(new_n209), .o1(new_n222));
  nanp02aa1n04x5               g127(.a(new_n221), .b(new_n222), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n220), .c(new_n180), .d(new_n184), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nand02aa1n06x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  nor042aa1n06x5               g134(.a(\b[21] ), .b(\a[22] ), .o1(new_n230));
  nand02aa1n06x5               g135(.a(\b[21] ), .b(\a[22] ), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n227), .c(new_n225), .d(new_n229), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(new_n225), .b(new_n229), .o1(new_n235));
  norb03aa1n02x5               g140(.a(new_n231), .b(new_n227), .c(new_n230), .out0(new_n236));
  nand02aa1n02x5               g141(.a(new_n235), .b(new_n236), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(new_n234), .b(new_n237), .o1(\s[22] ));
  nano23aa1d15x5               g143(.a(new_n230), .b(new_n227), .c(new_n231), .d(new_n228), .out0(new_n239));
  nanp03aa1n02x5               g144(.a(new_n201), .b(new_n219), .c(new_n239), .o1(new_n240));
  and002aa1n02x5               g145(.a(\b[21] ), .b(\a[22] ), .o(new_n241));
  oab012aa1n09x5               g146(.a(new_n241), .b(new_n227), .c(new_n230), .out0(new_n242));
  tech160nm_fiaoi012aa1n05x5   g147(.a(new_n242), .b(new_n223), .c(new_n239), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n240), .c(new_n180), .d(new_n184), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1d32x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  nand02aa1d28x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(new_n248));
  nor002aa1d32x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  nand02aa1d28x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  nanb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n246), .c(new_n244), .d(new_n248), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(new_n244), .b(new_n248), .o1(new_n253));
  nona22aa1d36x5               g158(.a(new_n250), .b(new_n249), .c(new_n246), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n253), .b(new_n255), .o1(new_n256));
  nanp02aa1n02x5               g161(.a(new_n252), .b(new_n256), .o1(\s[24] ));
  inv000aa1d42x5               g162(.a(new_n219), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n239), .o1(new_n259));
  nano23aa1d15x5               g164(.a(new_n246), .b(new_n249), .c(new_n250), .d(new_n247), .out0(new_n260));
  nona23aa1n06x5               g165(.a(new_n201), .b(new_n260), .c(new_n259), .d(new_n258), .out0(new_n261));
  nand02aa1d04x5               g166(.a(new_n260), .b(new_n239), .o1(new_n262));
  aoi022aa1n12x5               g167(.a(new_n260), .b(new_n242), .c(new_n250), .d(new_n254), .o1(new_n263));
  aoai13aa1n12x5               g168(.a(new_n263), .b(new_n262), .c(new_n221), .d(new_n222), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n261), .c(new_n180), .d(new_n184), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  inv000aa1d42x5               g172(.a(new_n260), .o1(new_n268));
  nona32aa1n03x5               g173(.a(new_n194), .b(new_n268), .c(new_n259), .d(new_n220), .out0(new_n269));
  xnrc02aa1n12x5               g174(.a(\b[24] ), .b(\a[25] ), .out0(new_n270));
  aoi012aa1n03x5               g175(.a(new_n270), .b(new_n269), .c(new_n265), .o1(new_n271));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n270), .o1(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[25] ), .b(\a[26] ), .out0(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n272), .c(new_n266), .d(new_n273), .o1(new_n275));
  oabi12aa1n02x5               g180(.a(new_n274), .b(\a[25] ), .c(\b[24] ), .out0(new_n276));
  oai012aa1n03x5               g181(.a(new_n275), .b(new_n271), .c(new_n276), .o1(\s[26] ));
  nor002aa1n02x5               g182(.a(new_n274), .b(new_n270), .o1(new_n278));
  nano32aa1n03x7               g183(.a(new_n220), .b(new_n278), .c(new_n239), .d(new_n260), .out0(new_n279));
  inv020aa1n02x5               g184(.a(new_n279), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(\b[25] ), .b(\a[26] ), .o1(new_n281));
  aoi022aa1n06x5               g186(.a(new_n264), .b(new_n278), .c(new_n281), .d(new_n276), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n280), .c(new_n180), .d(new_n184), .o1(new_n283));
  xorb03aa1n03x5               g188(.a(new_n283), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g189(.a(\b[26] ), .b(\a[27] ), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[27] ), .b(\b[26] ), .out0(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[27] ), .b(\a[28] ), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n285), .c(new_n283), .d(new_n286), .o1(new_n288));
  nand22aa1n03x5               g193(.a(new_n264), .b(new_n278), .o1(new_n289));
  oai012aa1n02x5               g194(.a(new_n281), .b(new_n274), .c(new_n272), .o1(new_n290));
  nand02aa1d04x5               g195(.a(new_n289), .b(new_n290), .o1(new_n291));
  aoai13aa1n02x7               g196(.a(new_n286), .b(new_n291), .c(new_n194), .d(new_n279), .o1(new_n292));
  norp02aa1n02x5               g197(.a(new_n287), .b(new_n285), .o1(new_n293));
  nand42aa1n02x5               g198(.a(new_n292), .b(new_n293), .o1(new_n294));
  nanp02aa1n03x5               g199(.a(new_n288), .b(new_n294), .o1(\s[28] ));
  xorc02aa1n12x5               g200(.a(\a[29] ), .b(\b[28] ), .out0(new_n296));
  inv000aa1d42x5               g201(.a(new_n296), .o1(new_n297));
  norb02aa1n02x5               g202(.a(new_n286), .b(new_n287), .out0(new_n298));
  aoi012aa1n02x5               g203(.a(new_n293), .b(\a[28] ), .c(\b[27] ), .o1(new_n299));
  aoai13aa1n02x7               g204(.a(new_n297), .b(new_n299), .c(new_n283), .d(new_n298), .o1(new_n300));
  aoai13aa1n02x7               g205(.a(new_n298), .b(new_n291), .c(new_n194), .d(new_n279), .o1(new_n301));
  nona22aa1n03x5               g206(.a(new_n301), .b(new_n299), .c(new_n297), .out0(new_n302));
  nanp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[29] ));
  xorb03aa1n02x5               g208(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g209(.a(new_n287), .b(new_n286), .c(new_n296), .out0(new_n305));
  aoai13aa1n06x5               g210(.a(new_n305), .b(new_n291), .c(new_n194), .d(new_n279), .o1(new_n306));
  norp02aa1n02x5               g211(.a(\b[28] ), .b(\a[29] ), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n307), .b(new_n299), .c(new_n296), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .out0(new_n309));
  oai012aa1n02x5               g214(.a(new_n309), .b(\b[28] ), .c(\a[29] ), .o1(new_n310));
  aoi012aa1n02x5               g215(.a(new_n310), .b(new_n299), .c(new_n296), .o1(new_n311));
  nanp02aa1n03x5               g216(.a(new_n306), .b(new_n311), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n309), .c(new_n306), .d(new_n308), .o1(\s[30] ));
  nano23aa1n02x4               g218(.a(new_n297), .b(new_n287), .c(new_n309), .d(new_n286), .out0(new_n314));
  aoi012aa1n02x5               g219(.a(new_n311), .b(\a[30] ), .c(\b[29] ), .o1(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[30] ), .b(\a[31] ), .out0(new_n316));
  aoai13aa1n02x7               g221(.a(new_n316), .b(new_n315), .c(new_n283), .d(new_n314), .o1(new_n317));
  aoai13aa1n03x5               g222(.a(new_n314), .b(new_n291), .c(new_n194), .d(new_n279), .o1(new_n318));
  nona22aa1n03x5               g223(.a(new_n318), .b(new_n315), .c(new_n316), .out0(new_n319));
  nanp02aa1n03x5               g224(.a(new_n317), .b(new_n319), .o1(\s[31] ));
  xnbna2aa1n03x5               g225(.a(new_n102), .b(new_n105), .c(new_n107), .out0(\s[3] ));
  orn002aa1n02x5               g226(.a(new_n102), .b(new_n106), .o(new_n322));
  xobna2aa1n03x5               g227(.a(new_n103), .b(new_n322), .c(new_n107), .out0(\s[4] ));
  xorb03aa1n02x5               g228(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g229(.a(new_n115), .b(new_n114), .out0(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n112), .c(new_n109), .d(new_n113), .o1(new_n326));
  aoi112aa1n02x5               g231(.a(new_n112), .b(new_n325), .c(new_n109), .d(new_n113), .o1(new_n327));
  nanb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(\s[6] ));
  oaib12aa1n02x5               g233(.a(new_n118), .b(new_n116), .c(new_n109), .out0(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g235(.a(new_n111), .b(new_n329), .out0(new_n331));
  xobna2aa1n03x5               g236(.a(new_n110), .b(new_n331), .c(new_n120), .out0(\s[8] ));
  xnbna2aa1n03x5               g237(.a(new_n124), .b(new_n190), .c(new_n192), .out0(\s[9] ));
endmodule


