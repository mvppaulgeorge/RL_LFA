// Benchmark "adder" written by ABC on Thu Jul 11 11:10:17 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n290, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n351, new_n354,
    new_n356, new_n358;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  xorc02aa1n02x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n02x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nanb03aa1n02x5               g011(.a(new_n102), .b(new_n103), .c(new_n106), .out0(new_n107));
  160nm_ficinv00aa1n08x5       g012(.clk(\a[4] ), .clkout(new_n108));
  160nm_ficinv00aa1n08x5       g013(.clk(\b[3] ), .clkout(new_n109));
  oaoi03aa1n02x5               g014(.a(new_n108), .b(new_n109), .c(new_n104), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n02x4               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nona22aa1n02x4               g022(.a(new_n115), .b(new_n116), .c(new_n117), .out0(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\a[5] ), .clkout(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\b[4] ), .clkout(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[6] ), .b(\b[5] ), .c(new_n121), .o1(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\a[7] ), .clkout(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(\b[6] ), .clkout(new_n124));
  aoai13aa1n02x5               g029(.a(new_n112), .b(new_n111), .c(new_n123), .d(new_n124), .o1(new_n125));
  aobi12aa1n02x5               g030(.a(new_n125), .b(new_n115), .c(new_n122), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n118), .c(new_n107), .d(new_n110), .o1(new_n127));
  oaoi03aa1n02x5               g032(.a(new_n97), .b(new_n98), .c(new_n127), .o1(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  norp02aa1n02x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[8] ), .b(\a[9] ), .o1(new_n134));
  norp02aa1n02x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nano23aa1n02x4               g041(.a(new_n133), .b(new_n135), .c(new_n136), .d(new_n134), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n136), .b(new_n135), .c(new_n97), .d(new_n98), .o1(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n138), .clkout(new_n139));
  aoai13aa1n02x5               g044(.a(new_n132), .b(new_n139), .c(new_n127), .d(new_n137), .o1(new_n140));
  aoi112aa1n02x5               g045(.a(new_n132), .b(new_n139), .c(new_n127), .d(new_n137), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(\s[11] ));
  oai012aa1n02x5               g047(.a(new_n140), .b(\b[10] ), .c(\a[11] ), .o1(new_n143));
  xorb03aa1n02x5               g048(.a(new_n143), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nanp02aa1n02x5               g049(.a(new_n109), .b(new_n108), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(\b[3] ), .b(\a[4] ), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n145), .b(new_n146), .o1(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(\a[3] ), .clkout(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(\b[2] ), .clkout(new_n149));
  nanp02aa1n02x5               g054(.a(new_n149), .b(new_n148), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n105), .o1(new_n151));
  oai013aa1n02x4               g056(.a(new_n110), .b(new_n102), .c(new_n147), .d(new_n151), .o1(new_n152));
  nona23aa1n02x4               g057(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n153));
  norp03aa1n02x5               g058(.a(new_n153), .b(new_n116), .c(new_n117), .o1(new_n154));
  oaib12aa1n02x5               g059(.a(new_n125), .b(new_n153), .c(new_n122), .out0(new_n155));
  norp02aa1n02x5               g060(.a(\b[11] ), .b(\a[12] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[11] ), .b(\a[12] ), .o1(new_n157));
  nano23aa1n02x4               g062(.a(new_n130), .b(new_n156), .c(new_n157), .d(new_n131), .out0(new_n158));
  nanp02aa1n02x5               g063(.a(new_n158), .b(new_n137), .o1(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n155), .c(new_n152), .d(new_n154), .o1(new_n161));
  oai012aa1n02x5               g066(.a(new_n157), .b(new_n156), .c(new_n130), .o1(new_n162));
  aobi12aa1n02x5               g067(.a(new_n162), .b(new_n158), .c(new_n139), .out0(new_n163));
  norp02aa1n02x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n166), .b(new_n161), .c(new_n163), .out0(\s[13] ));
  norp03aa1n02x5               g072(.a(new_n102), .b(new_n147), .c(new_n151), .o1(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n110), .clkout(new_n169));
  oai012aa1n02x5               g074(.a(new_n154), .b(new_n168), .c(new_n169), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n163), .b(new_n159), .c(new_n170), .d(new_n126), .o1(new_n171));
  aoi012aa1n02x5               g076(.a(new_n164), .b(new_n171), .c(new_n165), .o1(new_n172));
  xnrb03aa1n02x5               g077(.a(new_n172), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  nona23aa1n02x4               g080(.a(new_n175), .b(new_n165), .c(new_n164), .d(new_n174), .out0(new_n176));
  oai012aa1n02x5               g081(.a(new_n175), .b(new_n174), .c(new_n164), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n176), .c(new_n161), .d(new_n163), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  nanb02aa1n02x5               g086(.a(new_n180), .b(new_n181), .out0(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n182), .clkout(new_n183));
  norp02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  nanb02aa1n02x5               g090(.a(new_n184), .b(new_n185), .out0(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(new_n186), .clkout(new_n187));
  aoi112aa1n02x5               g092(.a(new_n187), .b(new_n180), .c(new_n178), .d(new_n183), .o1(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(new_n180), .clkout(new_n189));
  nano23aa1n02x4               g094(.a(new_n164), .b(new_n174), .c(new_n175), .d(new_n165), .out0(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(new_n177), .clkout(new_n191));
  aoai13aa1n02x5               g096(.a(new_n183), .b(new_n191), .c(new_n171), .d(new_n190), .o1(new_n192));
  aoi012aa1n02x5               g097(.a(new_n186), .b(new_n192), .c(new_n189), .o1(new_n193));
  norp02aa1n02x5               g098(.a(new_n193), .b(new_n188), .o1(\s[16] ));
  nona22aa1n02x4               g099(.a(new_n190), .b(new_n186), .c(new_n182), .out0(new_n195));
  norp02aa1n02x5               g100(.a(new_n195), .b(new_n159), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n155), .c(new_n152), .d(new_n154), .o1(new_n197));
  nona23aa1n02x4               g102(.a(new_n157), .b(new_n131), .c(new_n130), .d(new_n156), .out0(new_n198));
  oai012aa1n02x5               g103(.a(new_n162), .b(new_n198), .c(new_n138), .o1(new_n199));
  nona23aa1n02x4               g104(.a(new_n185), .b(new_n181), .c(new_n180), .d(new_n184), .out0(new_n200));
  norp02aa1n02x5               g105(.a(new_n200), .b(new_n176), .o1(new_n201));
  oai012aa1n02x5               g106(.a(new_n185), .b(new_n184), .c(new_n180), .o1(new_n202));
  oai012aa1n02x5               g107(.a(new_n202), .b(new_n200), .c(new_n177), .o1(new_n203));
  aoi012aa1n02x5               g108(.a(new_n203), .b(new_n199), .c(new_n201), .o1(new_n204));
  xorc02aa1n02x5               g109(.a(\a[17] ), .b(\b[16] ), .out0(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n197), .c(new_n204), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g111(.clk(\a[17] ), .clkout(new_n207));
  160nm_ficinv00aa1n08x5       g112(.clk(\b[16] ), .clkout(new_n208));
  nanp02aa1n02x5               g113(.a(new_n208), .b(new_n207), .o1(new_n209));
  oabi12aa1n02x5               g114(.a(new_n203), .b(new_n163), .c(new_n195), .out0(new_n210));
  aoai13aa1n02x5               g115(.a(new_n205), .b(new_n210), .c(new_n127), .d(new_n196), .o1(new_n211));
  norp02aa1n02x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  xobna2aa1n03x5               g119(.a(new_n214), .b(new_n211), .c(new_n209), .out0(\s[18] ));
  nanp02aa1n02x5               g120(.a(\b[16] ), .b(\a[17] ), .o1(new_n216));
  nano22aa1n02x4               g121(.a(new_n214), .b(new_n209), .c(new_n216), .out0(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  aoai13aa1n02x5               g123(.a(new_n213), .b(new_n212), .c(new_n207), .d(new_n208), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n218), .c(new_n197), .d(new_n204), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  nanb02aa1n02x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  norp02aa1n02x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  nanb02aa1n02x5               g133(.a(new_n227), .b(new_n228), .out0(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n229), .clkout(new_n230));
  aoi112aa1n02x5               g135(.a(new_n223), .b(new_n230), .c(new_n220), .d(new_n226), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n223), .clkout(new_n232));
  nona23aa1n02x4               g137(.a(new_n190), .b(new_n137), .c(new_n200), .d(new_n198), .out0(new_n233));
  aoai13aa1n02x5               g138(.a(new_n204), .b(new_n233), .c(new_n170), .d(new_n126), .o1(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n219), .clkout(new_n235));
  aoai13aa1n02x5               g140(.a(new_n226), .b(new_n235), .c(new_n234), .d(new_n217), .o1(new_n236));
  aoi012aa1n02x5               g141(.a(new_n229), .b(new_n236), .c(new_n232), .o1(new_n237));
  norp02aa1n02x5               g142(.a(new_n237), .b(new_n231), .o1(\s[20] ));
  nano23aa1n02x4               g143(.a(new_n223), .b(new_n227), .c(new_n228), .d(new_n224), .out0(new_n239));
  nanp02aa1n02x5               g144(.a(new_n217), .b(new_n239), .o1(new_n240));
  nona23aa1n02x4               g145(.a(new_n228), .b(new_n224), .c(new_n223), .d(new_n227), .out0(new_n241));
  oai012aa1n02x5               g146(.a(new_n228), .b(new_n227), .c(new_n223), .o1(new_n242));
  oai012aa1n02x5               g147(.a(new_n242), .b(new_n241), .c(new_n219), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n240), .c(new_n197), .d(new_n204), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g151(.a(\b[20] ), .b(\a[21] ), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(\b[20] ), .b(\a[21] ), .o1(new_n248));
  nanb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  norp02aa1n02x5               g155(.a(\b[21] ), .b(\a[22] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  nanb02aa1n02x5               g157(.a(new_n251), .b(new_n252), .out0(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n253), .clkout(new_n254));
  aoi112aa1n02x5               g159(.a(new_n247), .b(new_n254), .c(new_n245), .d(new_n250), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n247), .clkout(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n240), .clkout(new_n257));
  aoai13aa1n02x5               g162(.a(new_n250), .b(new_n243), .c(new_n234), .d(new_n257), .o1(new_n258));
  aoi012aa1n02x5               g163(.a(new_n253), .b(new_n258), .c(new_n256), .o1(new_n259));
  norp02aa1n02x5               g164(.a(new_n259), .b(new_n255), .o1(\s[22] ));
  nano23aa1n02x4               g165(.a(new_n247), .b(new_n251), .c(new_n252), .d(new_n248), .out0(new_n261));
  nanp03aa1n02x5               g166(.a(new_n217), .b(new_n239), .c(new_n261), .o1(new_n262));
  oaoi03aa1n02x5               g167(.a(\a[22] ), .b(\b[21] ), .c(new_n256), .o1(new_n263));
  aoi012aa1n02x5               g168(.a(new_n263), .b(new_n243), .c(new_n261), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n262), .c(new_n197), .d(new_n204), .o1(new_n265));
  xorb03aa1n02x5               g170(.a(new_n265), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  nanp02aa1n02x5               g172(.a(\b[22] ), .b(\a[23] ), .o1(new_n268));
  norb02aa1n02x5               g173(.a(new_n268), .b(new_n267), .out0(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[23] ), .b(\a[24] ), .out0(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  aoi112aa1n02x5               g176(.a(new_n267), .b(new_n271), .c(new_n265), .d(new_n269), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n267), .clkout(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n262), .clkout(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n264), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n269), .b(new_n275), .c(new_n234), .d(new_n274), .o1(new_n276));
  aoi012aa1n02x5               g181(.a(new_n270), .b(new_n276), .c(new_n273), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n277), .b(new_n272), .o1(\s[24] ));
  nano32aa1n02x4               g183(.a(new_n240), .b(new_n271), .c(new_n261), .d(new_n269), .out0(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n279), .clkout(new_n280));
  nanp02aa1n02x5               g185(.a(new_n239), .b(new_n235), .o1(new_n281));
  nanp03aa1n02x5               g186(.a(new_n271), .b(new_n261), .c(new_n269), .o1(new_n282));
  nano22aa1n02x4               g187(.a(new_n270), .b(new_n273), .c(new_n268), .out0(new_n283));
  oaoi03aa1n02x5               g188(.a(\a[24] ), .b(\b[23] ), .c(new_n273), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n284), .b(new_n283), .c(new_n263), .o1(new_n285));
  aoai13aa1n02x5               g190(.a(new_n285), .b(new_n282), .c(new_n281), .d(new_n242), .o1(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n286), .clkout(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n280), .c(new_n197), .d(new_n204), .o1(new_n288));
  xorb03aa1n02x5               g193(.a(new_n288), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g194(.a(\b[24] ), .b(\a[25] ), .o1(new_n290));
  xorc02aa1n02x5               g195(.a(\a[25] ), .b(\b[24] ), .out0(new_n291));
  xorc02aa1n02x5               g196(.a(\a[26] ), .b(\b[25] ), .out0(new_n292));
  aoi112aa1n02x5               g197(.a(new_n290), .b(new_n292), .c(new_n288), .d(new_n291), .o1(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n290), .clkout(new_n294));
  aoai13aa1n02x5               g199(.a(new_n291), .b(new_n286), .c(new_n234), .d(new_n279), .o1(new_n295));
  160nm_ficinv00aa1n08x5       g200(.clk(new_n292), .clkout(new_n296));
  aoi012aa1n02x5               g201(.a(new_n296), .b(new_n295), .c(new_n294), .o1(new_n297));
  norp02aa1n02x5               g202(.a(new_n297), .b(new_n293), .o1(\s[26] ));
  nano32aa1n02x4               g203(.a(new_n262), .b(new_n292), .c(new_n283), .d(new_n291), .out0(new_n299));
  aoai13aa1n02x5               g204(.a(new_n299), .b(new_n210), .c(new_n127), .d(new_n196), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(\b[25] ), .b(\a[26] ), .o1(new_n301));
  and002aa1n02x5               g206(.a(new_n292), .b(new_n291), .o(new_n302));
  oai022aa1n02x5               g207(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n303));
  aoi022aa1n02x5               g208(.a(new_n286), .b(new_n302), .c(new_n301), .d(new_n303), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[27] ), .b(\b[26] ), .out0(new_n305));
  xnbna2aa1n03x5               g210(.a(new_n305), .b(new_n300), .c(new_n304), .out0(\s[27] ));
  norp02aa1n02x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  160nm_ficinv00aa1n08x5       g212(.clk(new_n307), .clkout(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n305), .clkout(new_n309));
  aoi012aa1n02x5               g214(.a(new_n309), .b(new_n300), .c(new_n304), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[27] ), .b(\a[28] ), .out0(new_n311));
  nano22aa1n02x4               g216(.a(new_n310), .b(new_n308), .c(new_n311), .out0(new_n312));
  nona23aa1n02x4               g217(.a(new_n252), .b(new_n248), .c(new_n247), .d(new_n251), .out0(new_n313));
  norb03aa1n02x5               g218(.a(new_n269), .b(new_n313), .c(new_n270), .out0(new_n314));
  nanp02aa1n02x5               g219(.a(new_n314), .b(new_n243), .o1(new_n315));
  160nm_ficinv00aa1n08x5       g220(.clk(new_n302), .clkout(new_n316));
  nanp02aa1n02x5               g221(.a(new_n303), .b(new_n301), .o1(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n316), .c(new_n315), .d(new_n285), .o1(new_n318));
  aoai13aa1n02x5               g223(.a(new_n305), .b(new_n318), .c(new_n234), .d(new_n299), .o1(new_n319));
  aoi012aa1n02x5               g224(.a(new_n311), .b(new_n319), .c(new_n308), .o1(new_n320));
  norp02aa1n02x5               g225(.a(new_n320), .b(new_n312), .o1(\s[28] ));
  norb02aa1n02x5               g226(.a(new_n305), .b(new_n311), .out0(new_n322));
  aoai13aa1n02x5               g227(.a(new_n322), .b(new_n318), .c(new_n234), .d(new_n299), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[28] ), .b(\b[27] ), .c(new_n308), .carry(new_n324));
  xnrc02aa1n02x5               g229(.a(\b[28] ), .b(\a[29] ), .out0(new_n325));
  aoi012aa1n02x5               g230(.a(new_n325), .b(new_n323), .c(new_n324), .o1(new_n326));
  160nm_ficinv00aa1n08x5       g231(.clk(new_n322), .clkout(new_n327));
  aoi012aa1n02x5               g232(.a(new_n327), .b(new_n300), .c(new_n304), .o1(new_n328));
  nano22aa1n02x4               g233(.a(new_n328), .b(new_n324), .c(new_n325), .out0(new_n329));
  norp02aa1n02x5               g234(.a(new_n326), .b(new_n329), .o1(\s[29] ));
  xorb03aa1n02x5               g235(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g236(.a(new_n305), .b(new_n325), .c(new_n311), .out0(new_n332));
  aoai13aa1n02x5               g237(.a(new_n332), .b(new_n318), .c(new_n234), .d(new_n299), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[29] ), .b(\b[28] ), .c(new_n324), .carry(new_n334));
  xnrc02aa1n02x5               g239(.a(\b[29] ), .b(\a[30] ), .out0(new_n335));
  aoi012aa1n02x5               g240(.a(new_n335), .b(new_n333), .c(new_n334), .o1(new_n336));
  160nm_ficinv00aa1n08x5       g241(.clk(new_n332), .clkout(new_n337));
  aoi012aa1n02x5               g242(.a(new_n337), .b(new_n300), .c(new_n304), .o1(new_n338));
  nano22aa1n02x4               g243(.a(new_n338), .b(new_n334), .c(new_n335), .out0(new_n339));
  norp02aa1n02x5               g244(.a(new_n336), .b(new_n339), .o1(\s[30] ));
  nona32aa1n02x4               g245(.a(new_n305), .b(new_n335), .c(new_n325), .d(new_n311), .out0(new_n341));
  aoi012aa1n02x5               g246(.a(new_n341), .b(new_n300), .c(new_n304), .o1(new_n342));
  oao003aa1n02x5               g247(.a(\a[30] ), .b(\b[29] ), .c(new_n334), .carry(new_n343));
  xnrc02aa1n02x5               g248(.a(\b[30] ), .b(\a[31] ), .out0(new_n344));
  nano22aa1n02x4               g249(.a(new_n342), .b(new_n343), .c(new_n344), .out0(new_n345));
  160nm_ficinv00aa1n08x5       g250(.clk(new_n341), .clkout(new_n346));
  aoai13aa1n02x5               g251(.a(new_n346), .b(new_n318), .c(new_n234), .d(new_n299), .o1(new_n347));
  aoi012aa1n02x5               g252(.a(new_n344), .b(new_n347), .c(new_n343), .o1(new_n348));
  norp02aa1n02x5               g253(.a(new_n348), .b(new_n345), .o1(\s[31] ));
  xnbna2aa1n03x5               g254(.a(new_n102), .b(new_n105), .c(new_n150), .out0(\s[3] ));
  oaoi03aa1n02x5               g255(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n351));
  xorb03aa1n02x5               g256(.a(new_n351), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g257(.a(new_n152), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g258(.a(new_n119), .b(new_n120), .c(new_n152), .o1(new_n354));
  xnrb03aa1n02x5               g259(.a(new_n354), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g260(.a(\a[6] ), .b(\b[5] ), .c(new_n354), .o1(new_n356));
  xorb03aa1n02x5               g261(.a(new_n356), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g262(.a(new_n123), .b(new_n124), .c(new_n356), .o1(new_n358));
  xnrb03aa1n02x5               g263(.a(new_n358), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g264(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


