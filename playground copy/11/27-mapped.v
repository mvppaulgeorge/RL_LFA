// Benchmark "adder" written by ABC on Thu Jul 11 11:43:25 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n322, new_n324, new_n325, new_n327;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n02x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  norb02aa1n02x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  norp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norb02aa1n02x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .out0(new_n116));
  xorc02aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .out0(new_n117));
  nano32aa1n02x4               g022(.a(new_n116), .b(new_n117), .c(new_n112), .d(new_n115), .out0(new_n118));
  nanp02aa1n02x5               g023(.a(new_n118), .b(new_n109), .o1(new_n119));
  aoi112aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  oab012aa1n02x4               g025(.a(new_n110), .b(\a[5] ), .c(\b[4] ), .out0(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(new_n121), .clkout(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(new_n113), .clkout(new_n123));
  nano32aa1n02x4               g028(.a(new_n116), .b(new_n123), .c(new_n114), .d(new_n111), .out0(new_n124));
  aoi112aa1n02x5               g029(.a(new_n113), .b(new_n120), .c(new_n124), .d(new_n122), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanb02aa1n02x5               g031(.a(new_n97), .b(new_n126), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n98), .b(new_n127), .c(new_n119), .d(new_n125), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nona23aa1n02x4               g036(.a(new_n126), .b(new_n131), .c(new_n130), .d(new_n97), .out0(new_n132));
  oai012aa1n02x5               g037(.a(new_n131), .b(new_n97), .c(new_n130), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n132), .c(new_n119), .d(new_n125), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n136), .b(new_n134), .c(new_n137), .o1(new_n138));
  xnrb03aa1n02x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nona23aa1n02x4               g046(.a(new_n141), .b(new_n137), .c(new_n136), .d(new_n140), .out0(new_n142));
  160nm_fiao0012aa1n02p5x5     g047(.a(new_n140), .b(new_n136), .c(new_n141), .o(new_n143));
  oabi12aa1n02x5               g048(.a(new_n143), .b(new_n142), .c(new_n133), .out0(new_n144));
  160nm_ficinv00aa1n08x5       g049(.clk(new_n144), .clkout(new_n145));
  nanb02aa1n02x5               g050(.a(new_n130), .b(new_n131), .out0(new_n146));
  nano23aa1n02x4               g051(.a(new_n136), .b(new_n140), .c(new_n141), .d(new_n137), .out0(new_n147));
  nona22aa1n02x4               g052(.a(new_n147), .b(new_n146), .c(new_n127), .out0(new_n148));
  aoai13aa1n02x5               g053(.a(new_n145), .b(new_n148), .c(new_n119), .d(new_n125), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g055(.clk(\a[14] ), .clkout(new_n151));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  xnrc02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .out0(new_n153));
  aoib12aa1n02x5               g058(.a(new_n152), .b(new_n149), .c(new_n153), .out0(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(new_n151), .out0(\s[14] ));
  norp02aa1n02x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nanb02aa1n02x5               g062(.a(new_n156), .b(new_n157), .out0(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(\b[13] ), .clkout(new_n160));
  oaoi03aa1n02x5               g065(.a(new_n151), .b(new_n160), .c(new_n152), .o1(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  xnrc02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .out0(new_n163));
  norp02aa1n02x5               g068(.a(new_n163), .b(new_n153), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n159), .b(new_n162), .c(new_n149), .d(new_n164), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n162), .b(new_n159), .c(new_n149), .d(new_n164), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  oai112aa1n02x5               g075(.a(new_n165), .b(new_n170), .c(\b[14] ), .d(\a[15] ), .o1(new_n171));
  oaoi13aa1n02x5               g076(.a(new_n170), .b(new_n165), .c(\a[15] ), .d(\b[14] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[16] ));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n111), .clkout(new_n174));
  xorc02aa1n02x5               g079(.a(\a[7] ), .b(\b[6] ), .out0(new_n175));
  nona23aa1n02x4               g080(.a(new_n175), .b(new_n115), .c(new_n121), .d(new_n174), .out0(new_n176));
  nona22aa1n02x4               g081(.a(new_n176), .b(new_n120), .c(new_n113), .out0(new_n177));
  nano23aa1n02x4               g082(.a(new_n156), .b(new_n168), .c(new_n169), .d(new_n157), .out0(new_n178));
  nano22aa1n02x4               g083(.a(new_n148), .b(new_n164), .c(new_n178), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n118), .d(new_n109), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n178), .b(new_n162), .c(new_n144), .d(new_n164), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n168), .b(new_n156), .c(new_n169), .o1(new_n182));
  nanp03aa1n02x5               g087(.a(new_n180), .b(new_n181), .c(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  nanp02aa1n02x5               g094(.a(new_n119), .b(new_n125), .o1(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(new_n178), .clkout(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(new_n133), .clkout(new_n192));
  aoai13aa1n02x5               g097(.a(new_n164), .b(new_n143), .c(new_n147), .d(new_n192), .o1(new_n193));
  aoai13aa1n02x5               g098(.a(new_n182), .b(new_n191), .c(new_n193), .d(new_n161), .o1(new_n194));
  xroi22aa1d04x5               g099(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n194), .c(new_n190), .d(new_n179), .o1(new_n196));
  oai022aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n02x5               g102(.a(new_n197), .b(new_n185), .c(\b[17] ), .out0(new_n198));
  norp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n201), .clkout(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n196), .c(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n199), .clkout(new_n205));
  aoi012aa1n02x5               g110(.a(new_n201), .b(new_n196), .c(new_n198), .o1(new_n206));
  norp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  nano22aa1n02x4               g114(.a(new_n206), .b(new_n205), .c(new_n209), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n187), .b(new_n186), .o1(new_n211));
  oaoi03aa1n02x5               g116(.a(\a[18] ), .b(\b[17] ), .c(new_n211), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n202), .b(new_n212), .c(new_n183), .d(new_n195), .o1(new_n213));
  aoi012aa1n02x5               g118(.a(new_n209), .b(new_n213), .c(new_n205), .o1(new_n214));
  norp02aa1n02x5               g119(.a(new_n214), .b(new_n210), .o1(\s[20] ));
  nano23aa1n02x4               g120(.a(new_n199), .b(new_n207), .c(new_n208), .d(new_n200), .out0(new_n216));
  nanp02aa1n02x5               g121(.a(new_n195), .b(new_n216), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n194), .c(new_n190), .d(new_n179), .o1(new_n219));
  nona23aa1n02x4               g124(.a(new_n208), .b(new_n200), .c(new_n199), .d(new_n207), .out0(new_n220));
  aoi012aa1n02x5               g125(.a(new_n207), .b(new_n199), .c(new_n208), .o1(new_n221));
  oai012aa1n02x5               g126(.a(new_n221), .b(new_n220), .c(new_n198), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n222), .clkout(new_n223));
  norp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  xnbna2aa1n03x5               g131(.a(new_n226), .b(new_n219), .c(new_n223), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n224), .clkout(new_n228));
  aobi12aa1n02x5               g133(.a(new_n226), .b(new_n219), .c(new_n223), .out0(new_n229));
  xnrc02aa1n02x5               g134(.a(\b[21] ), .b(\a[22] ), .out0(new_n230));
  nano22aa1n02x4               g135(.a(new_n229), .b(new_n228), .c(new_n230), .out0(new_n231));
  aoai13aa1n02x5               g136(.a(new_n226), .b(new_n222), .c(new_n183), .d(new_n218), .o1(new_n232));
  aoi012aa1n02x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .o1(new_n233));
  norp02aa1n02x5               g138(.a(new_n233), .b(new_n231), .o1(\s[22] ));
  nano22aa1n02x4               g139(.a(new_n230), .b(new_n228), .c(new_n225), .out0(new_n235));
  and003aa1n02x5               g140(.a(new_n195), .b(new_n235), .c(new_n216), .o(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n194), .c(new_n190), .d(new_n179), .o1(new_n237));
  oao003aa1n02x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n228), .carry(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  aoi012aa1n02x5               g144(.a(new_n239), .b(new_n222), .c(new_n235), .o1(new_n240));
  xnrc02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .out0(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n241), .clkout(new_n242));
  xnbna2aa1n03x5               g147(.a(new_n242), .b(new_n237), .c(new_n240), .out0(\s[23] ));
  norp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  aoi012aa1n02x5               g150(.a(new_n241), .b(new_n237), .c(new_n240), .o1(new_n246));
  xnrc02aa1n02x5               g151(.a(\b[23] ), .b(\a[24] ), .out0(new_n247));
  nano22aa1n02x4               g152(.a(new_n246), .b(new_n245), .c(new_n247), .out0(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n240), .clkout(new_n249));
  aoai13aa1n02x5               g154(.a(new_n242), .b(new_n249), .c(new_n183), .d(new_n236), .o1(new_n250));
  aoi012aa1n02x5               g155(.a(new_n247), .b(new_n250), .c(new_n245), .o1(new_n251));
  norp02aa1n02x5               g156(.a(new_n251), .b(new_n248), .o1(\s[24] ));
  norp02aa1n02x5               g157(.a(new_n247), .b(new_n241), .o1(new_n253));
  nano22aa1n02x4               g158(.a(new_n217), .b(new_n235), .c(new_n253), .out0(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n194), .c(new_n190), .d(new_n179), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n221), .clkout(new_n256));
  aoai13aa1n02x5               g161(.a(new_n235), .b(new_n256), .c(new_n216), .d(new_n212), .o1(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n253), .clkout(new_n258));
  oao003aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .c(new_n245), .carry(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n258), .c(new_n257), .d(new_n238), .o1(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n260), .clkout(new_n261));
  xnrc02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .out0(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n262), .clkout(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n255), .c(new_n261), .out0(\s[25] ));
  norp02aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n265), .clkout(new_n266));
  aoi012aa1n02x5               g171(.a(new_n262), .b(new_n255), .c(new_n261), .o1(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[25] ), .b(\a[26] ), .out0(new_n268));
  nano22aa1n02x4               g173(.a(new_n267), .b(new_n266), .c(new_n268), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n263), .b(new_n260), .c(new_n183), .d(new_n254), .o1(new_n270));
  aoi012aa1n02x5               g175(.a(new_n268), .b(new_n270), .c(new_n266), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n269), .o1(\s[26] ));
  norp02aa1n02x5               g177(.a(new_n268), .b(new_n262), .o1(new_n273));
  nano32aa1n02x4               g178(.a(new_n217), .b(new_n273), .c(new_n235), .d(new_n253), .out0(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n194), .c(new_n190), .d(new_n179), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n276));
  aobi12aa1n02x5               g181(.a(new_n276), .b(new_n260), .c(new_n273), .out0(new_n277));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  norb02aa1n02x5               g184(.a(new_n279), .b(new_n278), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n275), .c(new_n277), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n278), .clkout(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  aoai13aa1n02x5               g188(.a(new_n253), .b(new_n239), .c(new_n222), .d(new_n235), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n273), .clkout(new_n285));
  aoai13aa1n02x5               g190(.a(new_n276), .b(new_n285), .c(new_n284), .d(new_n259), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n279), .b(new_n286), .c(new_n183), .d(new_n274), .o1(new_n287));
  aoi012aa1n02x5               g192(.a(new_n283), .b(new_n287), .c(new_n282), .o1(new_n288));
  aobi12aa1n02x5               g193(.a(new_n279), .b(new_n275), .c(new_n277), .out0(new_n289));
  nano22aa1n02x4               g194(.a(new_n289), .b(new_n282), .c(new_n283), .out0(new_n290));
  norp02aa1n02x5               g195(.a(new_n288), .b(new_n290), .o1(\s[28] ));
  nano22aa1n02x4               g196(.a(new_n283), .b(new_n282), .c(new_n279), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n286), .c(new_n183), .d(new_n274), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n02x5               g201(.a(new_n292), .b(new_n275), .c(new_n277), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n296), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g205(.a(new_n280), .b(new_n295), .c(new_n283), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n286), .c(new_n183), .d(new_n274), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  aoi012aa1n02x5               g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  aobi12aa1n02x5               g210(.a(new_n301), .b(new_n275), .c(new_n277), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n306), .b(new_n303), .c(new_n304), .out0(new_n307));
  norp02aa1n02x5               g212(.a(new_n305), .b(new_n307), .o1(\s[30] ));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  norb03aa1n02x5               g214(.a(new_n292), .b(new_n304), .c(new_n295), .out0(new_n310));
  aobi12aa1n02x5               g215(.a(new_n310), .b(new_n275), .c(new_n277), .out0(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n312));
  nano22aa1n02x4               g217(.a(new_n311), .b(new_n309), .c(new_n312), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n310), .b(new_n286), .c(new_n183), .d(new_n274), .o1(new_n314));
  aoi012aa1n02x5               g219(.a(new_n309), .b(new_n314), .c(new_n312), .o1(new_n315));
  norp02aa1n02x5               g220(.a(new_n315), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  160nm_ficinv00aa1n08x5       g225(.clk(new_n109), .clkout(new_n321));
  oaoi03aa1n02x5               g226(.a(\a[5] ), .b(\b[4] ), .c(new_n321), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g228(.a(new_n175), .b(new_n110), .c(new_n322), .d(new_n112), .o1(new_n324));
  aoi112aa1n02x5               g229(.a(new_n175), .b(new_n110), .c(new_n322), .d(new_n112), .o1(new_n325));
  norb02aa1n02x5               g230(.a(new_n324), .b(new_n325), .out0(\s[7] ));
  orn002aa1n02x5               g231(.a(\a[7] ), .b(\b[6] ), .o(new_n327));
  xnbna2aa1n03x5               g232(.a(new_n115), .b(new_n324), .c(new_n327), .out0(\s[8] ));
  xobna2aa1n03x5               g233(.a(new_n127), .b(new_n119), .c(new_n125), .out0(\s[9] ));
endmodule


