// Benchmark "adder" written by ABC on Thu Jul 11 11:26:48 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n310, new_n312,
    new_n314, new_n316;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  orn002aa1n02x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aob012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n02x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  norp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n02x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nanp03aa1n02x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  oai012aa1n02x5               g014(.a(new_n104), .b(new_n106), .c(new_n103), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n02x4               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  nanp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n118), .b(new_n117), .c(new_n119), .d(new_n116), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  and002aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o(new_n122));
  oab012aa1n02x4               g027(.a(new_n122), .b(new_n117), .c(new_n118), .out0(new_n123));
  oai022aa1n02x5               g028(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n124));
  aoi022aa1n02x5               g029(.a(new_n115), .b(new_n123), .c(new_n112), .d(new_n124), .o1(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n121), .c(new_n109), .d(new_n110), .o1(new_n126));
  xnrc02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  aoib12aa1n02x5               g032(.a(new_n98), .b(new_n126), .c(new_n127), .out0(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  norp02aa1n02x5               g035(.a(new_n130), .b(new_n127), .o1(new_n131));
  orn002aa1n02x5               g036(.a(\a[9] ), .b(\b[8] ), .o(new_n132));
  oaoi03aa1n02x5               g037(.a(\a[10] ), .b(\b[9] ), .c(new_n132), .o1(new_n133));
  norp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n133), .c(new_n126), .d(new_n131), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n136), .b(new_n133), .c(new_n126), .d(new_n131), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g044(.clk(new_n134), .clkout(new_n140));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n140), .out0(\s[12] ));
  nano23aa1n02x4               g049(.a(new_n134), .b(new_n141), .c(new_n142), .d(new_n135), .out0(new_n145));
  oai022aa1n02x5               g050(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n146));
  aoi022aa1n02x5               g051(.a(new_n145), .b(new_n133), .c(new_n142), .d(new_n146), .o1(new_n147));
  nanp03aa1n02x5               g052(.a(new_n126), .b(new_n131), .c(new_n145), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n148), .b(new_n147), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g055(.clk(\a[14] ), .clkout(new_n151));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n149), .c(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(new_n151), .out0(\s[14] ));
  norp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1n02x4               g062(.a(new_n152), .b(new_n156), .c(new_n157), .d(new_n153), .out0(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  oa0012aa1n02x5               g064(.a(new_n157), .b(new_n156), .c(new_n152), .o(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n159), .c(new_n148), .d(new_n147), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n164), .c(new_n162), .d(new_n166), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(new_n162), .b(new_n166), .o1(new_n171));
  nona22aa1n02x4               g076(.a(new_n171), .b(new_n169), .c(new_n164), .out0(new_n172));
  nanp02aa1n02x5               g077(.a(new_n172), .b(new_n170), .o1(\s[16] ));
  nano23aa1n02x4               g078(.a(new_n164), .b(new_n167), .c(new_n168), .d(new_n165), .out0(new_n174));
  nanp02aa1n02x5               g079(.a(new_n174), .b(new_n158), .o1(new_n175));
  nano22aa1n02x4               g080(.a(new_n175), .b(new_n131), .c(new_n145), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n126), .b(new_n176), .o1(new_n177));
  oai012aa1n02x5               g082(.a(new_n168), .b(new_n167), .c(new_n164), .o1(new_n178));
  aobi12aa1n02x5               g083(.a(new_n178), .b(new_n174), .c(new_n160), .out0(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n179), .clkout(new_n180));
  oab012aa1n02x4               g085(.a(new_n180), .b(new_n147), .c(new_n175), .out0(new_n181));
  xorc02aa1n02x5               g086(.a(\a[17] ), .b(\b[16] ), .out0(new_n182));
  xnbna2aa1n03x5               g087(.a(new_n182), .b(new_n177), .c(new_n181), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[17] ), .clkout(new_n184));
  nanb02aa1n02x5               g089(.a(\b[16] ), .b(new_n184), .out0(new_n185));
  oai012aa1n02x5               g090(.a(new_n179), .b(new_n147), .c(new_n175), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n182), .b(new_n186), .c(new_n126), .d(new_n176), .o1(new_n187));
  xnrc02aa1n02x5               g092(.a(\b[17] ), .b(\a[18] ), .out0(new_n188));
  xobna2aa1n03x5               g093(.a(new_n188), .b(new_n187), .c(new_n185), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[18] ), .clkout(new_n190));
  xroi22aa1d04x5               g095(.a(new_n184), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n191));
  aoai13aa1n02x5               g096(.a(new_n191), .b(new_n186), .c(new_n126), .d(new_n176), .o1(new_n192));
  oai022aa1n02x5               g097(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n193));
  oaib12aa1n02x5               g098(.a(new_n193), .b(new_n190), .c(\b[17] ), .out0(new_n194));
  norp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n197), .clkout(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n192), .c(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g105(.a(new_n192), .b(new_n194), .o1(new_n201));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n195), .c(new_n201), .d(new_n198), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(new_n201), .b(new_n198), .o1(new_n206));
  nona22aa1n02x4               g111(.a(new_n206), .b(new_n204), .c(new_n195), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n207), .b(new_n205), .o1(\s[20] ));
  nona23aa1n02x4               g113(.a(new_n203), .b(new_n196), .c(new_n195), .d(new_n202), .out0(new_n209));
  oai012aa1n02x5               g114(.a(new_n203), .b(new_n202), .c(new_n195), .o1(new_n210));
  oai012aa1n02x5               g115(.a(new_n210), .b(new_n209), .c(new_n194), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  nano23aa1n02x4               g117(.a(new_n195), .b(new_n202), .c(new_n203), .d(new_n196), .out0(new_n213));
  nanb03aa1n02x5               g118(.a(new_n188), .b(new_n213), .c(new_n182), .out0(new_n214));
  aoai13aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n177), .d(new_n181), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xnrc02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .out0(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  xnrc02aa1n02x5               g124(.a(\b[21] ), .b(\a[22] ), .out0(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n215), .d(new_n219), .o1(new_n221));
  nanp02aa1n02x5               g126(.a(new_n215), .b(new_n219), .o1(new_n222));
  nona22aa1n02x4               g127(.a(new_n222), .b(new_n220), .c(new_n217), .out0(new_n223));
  nanp02aa1n02x5               g128(.a(new_n223), .b(new_n221), .o1(\s[22] ));
  norp02aa1n02x5               g129(.a(new_n220), .b(new_n218), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(\a[22] ), .clkout(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\b[21] ), .clkout(new_n227));
  oao003aa1n02x5               g132(.a(new_n226), .b(new_n227), .c(new_n217), .carry(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n211), .c(new_n225), .o1(new_n229));
  nanp03aa1n02x5               g134(.a(new_n191), .b(new_n225), .c(new_n213), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n229), .b(new_n230), .c(new_n177), .d(new_n181), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xnrc02aa1n02x5               g139(.a(\b[23] ), .b(\a[24] ), .out0(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n231), .b(new_n234), .o1(new_n237));
  nona22aa1n02x4               g142(.a(new_n237), .b(new_n235), .c(new_n233), .out0(new_n238));
  nanp02aa1n02x5               g143(.a(new_n238), .b(new_n236), .o1(\s[24] ));
  norb02aa1n02x5               g144(.a(new_n234), .b(new_n235), .out0(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  nano32aa1n02x4               g146(.a(new_n241), .b(new_n191), .c(new_n225), .d(new_n213), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n186), .c(new_n126), .d(new_n176), .o1(new_n243));
  oaoi03aa1n02x5               g148(.a(\a[18] ), .b(\b[17] ), .c(new_n185), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n210), .clkout(new_n245));
  aoai13aa1n02x5               g150(.a(new_n225), .b(new_n245), .c(new_n213), .d(new_n244), .o1(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n228), .clkout(new_n247));
  orn002aa1n02x5               g152(.a(\a[23] ), .b(\b[22] ), .o(new_n248));
  oaoi03aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .c(new_n248), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n241), .c(new_n246), .d(new_n247), .o1(new_n251));
  nanb02aa1n02x5               g156(.a(new_n251), .b(new_n243), .out0(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  xorc02aa1n02x5               g160(.a(\a[26] ), .b(\b[25] ), .out0(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n256), .clkout(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n254), .c(new_n252), .d(new_n255), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n252), .b(new_n255), .o1(new_n259));
  nona22aa1n02x4               g164(.a(new_n259), .b(new_n257), .c(new_n254), .out0(new_n260));
  nanp02aa1n02x5               g165(.a(new_n260), .b(new_n258), .o1(\s[26] ));
  and002aa1n02x5               g166(.a(new_n256), .b(new_n255), .o(new_n262));
  nano22aa1n02x4               g167(.a(new_n230), .b(new_n262), .c(new_n240), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n186), .c(new_n126), .d(new_n176), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n251), .b(new_n262), .o1(new_n265));
  oai022aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n266));
  aob012aa1n02x5               g171(.a(new_n266), .b(\b[25] ), .c(\a[26] ), .out0(new_n267));
  nanp03aa1n02x5               g172(.a(new_n265), .b(new_n264), .c(new_n267), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n273));
  aobi12aa1n02x5               g178(.a(new_n263), .b(new_n177), .c(new_n181), .out0(new_n274));
  aoai13aa1n02x5               g179(.a(new_n240), .b(new_n228), .c(new_n211), .d(new_n225), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n262), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n267), .b(new_n276), .c(new_n275), .d(new_n250), .o1(new_n277));
  oai012aa1n02x5               g182(.a(new_n271), .b(new_n277), .c(new_n274), .o1(new_n278));
  nona22aa1n02x4               g183(.a(new_n278), .b(new_n272), .c(new_n270), .out0(new_n279));
  nanp02aa1n02x5               g184(.a(new_n273), .b(new_n279), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n271), .b(new_n272), .out0(new_n281));
  oai012aa1n02x5               g186(.a(new_n281), .b(new_n277), .c(new_n274), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n270), .clkout(new_n283));
  oaoi03aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .o1(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  nona22aa1n02x4               g190(.a(new_n282), .b(new_n284), .c(new_n285), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n285), .b(new_n284), .c(new_n268), .d(new_n281), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(new_n287), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n271), .b(new_n285), .c(new_n272), .out0(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n291));
  oaoi03aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[30] ), .b(\b[29] ), .out0(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n293), .clkout(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n268), .d(new_n290), .o1(new_n295));
  oai012aa1n02x5               g200(.a(new_n290), .b(new_n277), .c(new_n274), .o1(new_n296));
  nona22aa1n02x4               g201(.a(new_n296), .b(new_n292), .c(new_n294), .out0(new_n297));
  nanp02aa1n02x5               g202(.a(new_n295), .b(new_n297), .o1(\s[30] ));
  nano23aa1n02x4               g203(.a(new_n285), .b(new_n272), .c(new_n293), .d(new_n271), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n299), .b(new_n277), .c(new_n274), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(new_n292), .b(new_n293), .o1(new_n301));
  oai012aa1n02x5               g206(.a(new_n301), .b(\b[29] ), .c(\a[30] ), .o1(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nona22aa1n02x4               g208(.a(new_n300), .b(new_n302), .c(new_n303), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n303), .b(new_n302), .c(new_n268), .d(new_n299), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(new_n305), .b(new_n304), .o1(\s[31] ));
  xorb03aa1n02x5               g211(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g212(.a(new_n106), .b(new_n102), .c(new_n107), .o1(new_n308));
  xnrc02aa1n02x5               g213(.a(new_n308), .b(new_n105), .out0(\s[4] ));
  nanp02aa1n02x5               g214(.a(new_n109), .b(new_n110), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g216(.a(new_n118), .b(new_n310), .c(new_n119), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g218(.a(new_n123), .b(new_n310), .c(new_n120), .o(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g220(.a(new_n113), .b(new_n314), .c(new_n114), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


