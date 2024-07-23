// Benchmark "adder" written by ABC on Thu Jul 11 12:11:41 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n326, new_n329, new_n331, new_n332, new_n334;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nona23aa1n02x4               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nanb02aa1n02x5               g010(.a(new_n104), .b(new_n105), .out0(new_n106));
  160nm_ficinv00aa1n08x5       g011(.clk(\a[5] ), .clkout(new_n107));
  160nm_ficinv00aa1n08x5       g012(.clk(\b[4] ), .clkout(new_n108));
  nanp02aa1n02x5               g013(.a(new_n108), .b(new_n107), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(new_n109), .b(new_n110), .o1(new_n111));
  norp03aa1n02x5               g016(.a(new_n103), .b(new_n106), .c(new_n111), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  nanp02aa1n02x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[0] ), .b(\a[1] ), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[1] ), .b(\a[2] ), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n118), .b(new_n120), .c(new_n119), .o1(new_n121));
  aoi012aa1n02x5               g026(.a(new_n113), .b(new_n115), .c(new_n114), .o1(new_n122));
  oai012aa1n02x5               g027(.a(new_n122), .b(new_n117), .c(new_n121), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(new_n123), .b(new_n112), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n105), .b(new_n104), .c(new_n107), .d(new_n108), .o1(new_n125));
  160nm_fiao0012aa1n02p5x5     g030(.a(new_n99), .b(new_n101), .c(new_n100), .o(new_n126));
  oabi12aa1n02x5               g031(.a(new_n126), .b(new_n103), .c(new_n125), .out0(new_n127));
  160nm_ficinv00aa1n08x5       g032(.clk(new_n127), .clkout(new_n128));
  nanp02aa1n02x5               g033(.a(new_n124), .b(new_n128), .o1(new_n129));
  oaoi03aa1n02x5               g034(.a(new_n97), .b(new_n98), .c(new_n129), .o1(new_n130));
  xnrb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanb02aa1n02x5               g038(.a(new_n132), .b(new_n133), .out0(new_n134));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n134), .clkout(new_n135));
  norp02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n136), .c(new_n97), .d(new_n98), .o1(new_n138));
  norp02aa1n02x5               g043(.a(\b[8] ), .b(\a[9] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[8] ), .b(\a[9] ), .o1(new_n140));
  nano23aa1n02x4               g045(.a(new_n136), .b(new_n139), .c(new_n140), .d(new_n137), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n127), .c(new_n123), .d(new_n112), .o1(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n135), .b(new_n142), .c(new_n138), .out0(\s[11] ));
  and002aa1n02x5               g048(.a(\b[9] ), .b(\a[10] ), .o(new_n144));
  oab012aa1n02x4               g049(.a(new_n144), .b(new_n136), .c(new_n139), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n135), .b(new_n145), .c(new_n129), .d(new_n141), .o1(new_n146));
  oai012aa1n02x5               g051(.a(new_n146), .b(\b[10] ), .c(\a[11] ), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nano23aa1n02x4               g055(.a(new_n132), .b(new_n149), .c(new_n150), .d(new_n133), .out0(new_n151));
  and002aa1n02x5               g056(.a(new_n151), .b(new_n141), .o(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n127), .c(new_n123), .d(new_n112), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n149), .b(new_n132), .c(new_n150), .o1(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  aoi012aa1n02x5               g060(.a(new_n155), .b(new_n151), .c(new_n145), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(new_n153), .b(new_n156), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g063(.clk(\a[13] ), .clkout(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(\b[12] ), .clkout(new_n160));
  oaoi03aa1n02x5               g065(.a(new_n159), .b(new_n160), .c(new_n157), .o1(new_n161));
  xnrb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  norp02aa1n02x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nona23aa1n02x4               g071(.a(new_n166), .b(new_n164), .c(new_n163), .d(new_n165), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n166), .b(new_n165), .c(new_n159), .d(new_n160), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n167), .c(new_n153), .d(new_n156), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norp02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(new_n175), .clkout(new_n176));
  aoi112aa1n02x5               g081(.a(new_n176), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n176), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(\s[16] ));
  nano23aa1n02x4               g084(.a(new_n163), .b(new_n165), .c(new_n166), .d(new_n164), .out0(new_n180));
  nano23aa1n02x4               g085(.a(new_n171), .b(new_n173), .c(new_n174), .d(new_n172), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n181), .b(new_n180), .o1(new_n182));
  nano22aa1n02x4               g087(.a(new_n182), .b(new_n141), .c(new_n151), .out0(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n127), .c(new_n123), .d(new_n112), .o1(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(new_n173), .clkout(new_n185));
  nanb02aa1n02x5               g090(.a(new_n168), .b(new_n181), .out0(new_n186));
  aoi112aa1n02x5               g091(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(new_n187), .clkout(new_n188));
  nanp02aa1n02x5               g093(.a(new_n151), .b(new_n145), .o1(new_n189));
  aoi012aa1n02x5               g094(.a(new_n182), .b(new_n189), .c(new_n154), .o1(new_n190));
  nano32aa1n02x4               g095(.a(new_n190), .b(new_n188), .c(new_n186), .d(new_n185), .out0(new_n191));
  nanp02aa1n02x5               g096(.a(new_n191), .b(new_n184), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g098(.clk(\a[18] ), .clkout(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(\a[17] ), .clkout(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(\b[16] ), .clkout(new_n196));
  oaoi03aa1n02x5               g101(.a(new_n195), .b(new_n196), .c(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n194), .out0(\s[18] ));
  xroi22aa1d04x5               g103(.a(new_n195), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n199), .clkout(new_n200));
  oai022aa1n02x5               g105(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n201));
  oaib12aa1n02x5               g106(.a(new_n201), .b(new_n194), .c(\b[17] ), .out0(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n200), .c(new_n191), .d(new_n184), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  xnrc02aa1n02x5               g111(.a(\b[18] ), .b(\a[19] ), .out0(new_n207));
  160nm_ficinv00aa1n08x5       g112(.clk(new_n207), .clkout(new_n208));
  xnrc02aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .out0(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  aoi112aa1n02x5               g115(.a(new_n206), .b(new_n210), .c(new_n203), .d(new_n208), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n206), .clkout(new_n212));
  nanp02aa1n02x5               g117(.a(new_n203), .b(new_n208), .o1(new_n213));
  aoi012aa1n02x5               g118(.a(new_n209), .b(new_n213), .c(new_n212), .o1(new_n214));
  norp02aa1n02x5               g119(.a(new_n214), .b(new_n211), .o1(\s[20] ));
  norp02aa1n02x5               g120(.a(new_n209), .b(new_n207), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(new_n199), .b(new_n216), .o1(new_n217));
  oao003aa1n02x5               g122(.a(\a[20] ), .b(\b[19] ), .c(new_n212), .carry(new_n218));
  oai013aa1n02x4               g123(.a(new_n218), .b(new_n207), .c(new_n209), .d(new_n202), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n191), .d(new_n184), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n223), .clkout(new_n227));
  nanp02aa1n02x5               g132(.a(new_n221), .b(new_n224), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n225), .clkout(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n228), .c(new_n227), .o1(new_n230));
  norp02aa1n02x5               g135(.a(new_n230), .b(new_n226), .o1(\s[22] ));
  160nm_ficinv00aa1n08x5       g136(.clk(\a[21] ), .clkout(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(\a[22] ), .clkout(new_n233));
  xroi22aa1d04x5               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  oaoi03aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n227), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n219), .c(new_n234), .o1(new_n236));
  nanp03aa1n02x5               g141(.a(new_n234), .b(new_n199), .c(new_n216), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n236), .b(new_n237), .c(new_n191), .d(new_n184), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xorc02aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .out0(new_n242));
  aoi112aa1n02x5               g147(.a(new_n240), .b(new_n242), .c(new_n238), .d(new_n241), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n240), .clkout(new_n244));
  nanp02aa1n02x5               g149(.a(new_n238), .b(new_n241), .o1(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n242), .clkout(new_n246));
  aoi012aa1n02x5               g151(.a(new_n246), .b(new_n245), .c(new_n244), .o1(new_n247));
  norp02aa1n02x5               g152(.a(new_n247), .b(new_n243), .o1(\s[24] ));
  nano32aa1n02x4               g153(.a(new_n246), .b(new_n241), .c(new_n225), .d(new_n224), .out0(new_n249));
  nanp03aa1n02x5               g154(.a(new_n249), .b(new_n199), .c(new_n216), .o1(new_n250));
  nona22aa1n02x4               g155(.a(new_n210), .b(new_n207), .c(new_n202), .out0(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(\a[23] ), .clkout(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(\a[24] ), .clkout(new_n253));
  xroi22aa1d04x5               g158(.a(new_n252), .b(\b[22] ), .c(new_n253), .d(\b[23] ), .out0(new_n254));
  nanp02aa1n02x5               g159(.a(new_n254), .b(new_n234), .o1(new_n255));
  oaoi03aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n244), .o1(new_n256));
  aoi013aa1n02x4               g161(.a(new_n256), .b(new_n235), .c(new_n241), .d(new_n242), .o1(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n255), .c(new_n251), .d(new_n218), .o1(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n258), .clkout(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n250), .c(new_n191), .d(new_n184), .o1(new_n260));
  xorb03aa1n02x5               g165(.a(new_n260), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  xorc02aa1n02x5               g167(.a(\a[25] ), .b(\b[24] ), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .out0(new_n264));
  aoi112aa1n02x5               g169(.a(new_n262), .b(new_n264), .c(new_n260), .d(new_n263), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n262), .clkout(new_n266));
  nanp02aa1n02x5               g171(.a(new_n260), .b(new_n263), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n264), .clkout(new_n268));
  aoi012aa1n02x5               g173(.a(new_n268), .b(new_n267), .c(new_n266), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n269), .b(new_n265), .o1(\s[26] ));
  nanb02aa1n02x5               g175(.a(new_n171), .b(new_n172), .out0(new_n271));
  norp03aa1n02x5               g176(.a(new_n168), .b(new_n175), .c(new_n271), .o1(new_n272));
  nanb02aa1n02x5               g177(.a(new_n149), .b(new_n150), .out0(new_n273));
  norp03aa1n02x5               g178(.a(new_n138), .b(new_n273), .c(new_n134), .o1(new_n274));
  norp03aa1n02x5               g179(.a(new_n167), .b(new_n175), .c(new_n271), .o1(new_n275));
  oai012aa1n02x5               g180(.a(new_n275), .b(new_n274), .c(new_n155), .o1(new_n276));
  nona32aa1n02x4               g181(.a(new_n276), .b(new_n187), .c(new_n272), .d(new_n173), .out0(new_n277));
  and002aa1n02x5               g182(.a(new_n264), .b(new_n263), .o(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n278), .clkout(new_n279));
  norp03aa1n02x5               g184(.a(new_n279), .b(new_n255), .c(new_n217), .o1(new_n280));
  aoai13aa1n02x5               g185(.a(new_n280), .b(new_n277), .c(new_n129), .d(new_n183), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n282));
  aobi12aa1n02x5               g187(.a(new_n282), .b(new_n258), .c(new_n278), .out0(new_n283));
  norp02aa1n02x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(\b[26] ), .b(\a[27] ), .o1(new_n285));
  nanb02aa1n02x5               g190(.a(new_n284), .b(new_n285), .out0(new_n286));
  xobna2aa1n03x5               g191(.a(new_n286), .b(new_n281), .c(new_n283), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n284), .clkout(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .out0(new_n289));
  nanp02aa1n02x5               g194(.a(new_n249), .b(new_n219), .o1(new_n290));
  aoai13aa1n02x5               g195(.a(new_n282), .b(new_n279), .c(new_n290), .d(new_n257), .o1(new_n291));
  aoai13aa1n02x5               g196(.a(new_n285), .b(new_n291), .c(new_n192), .d(new_n280), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n289), .b(new_n292), .c(new_n288), .o1(new_n293));
  aoi022aa1n02x5               g198(.a(new_n281), .b(new_n283), .c(\b[26] ), .d(\a[27] ), .o1(new_n294));
  nano22aa1n02x4               g199(.a(new_n294), .b(new_n288), .c(new_n289), .out0(new_n295));
  norp02aa1n02x5               g200(.a(new_n293), .b(new_n295), .o1(\s[28] ));
  norp02aa1n02x5               g201(.a(new_n289), .b(new_n286), .o1(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n291), .c(new_n192), .d(new_n280), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[28] ), .b(\a[29] ), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n297), .clkout(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n281), .c(new_n283), .o1(new_n303));
  nano22aa1n02x4               g208(.a(new_n303), .b(new_n299), .c(new_n300), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n301), .b(new_n304), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n119), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norp03aa1n02x5               g211(.a(new_n300), .b(new_n289), .c(new_n286), .o1(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n291), .c(new_n192), .d(new_n280), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[29] ), .b(\b[28] ), .c(new_n299), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[29] ), .b(\a[30] ), .out0(new_n310));
  aoi012aa1n02x5               g215(.a(new_n310), .b(new_n308), .c(new_n309), .o1(new_n311));
  160nm_ficinv00aa1n08x5       g216(.clk(new_n307), .clkout(new_n312));
  aoi012aa1n02x5               g217(.a(new_n312), .b(new_n281), .c(new_n283), .o1(new_n313));
  nano22aa1n02x4               g218(.a(new_n313), .b(new_n309), .c(new_n310), .out0(new_n314));
  norp02aa1n02x5               g219(.a(new_n311), .b(new_n314), .o1(\s[30] ));
  xnrc02aa1n02x5               g220(.a(\b[30] ), .b(\a[31] ), .out0(new_n316));
  norb03aa1n02x5               g221(.a(new_n297), .b(new_n310), .c(new_n300), .out0(new_n317));
  160nm_ficinv00aa1n08x5       g222(.clk(new_n317), .clkout(new_n318));
  aoi012aa1n02x5               g223(.a(new_n318), .b(new_n281), .c(new_n283), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .c(new_n309), .carry(new_n320));
  nano22aa1n02x4               g225(.a(new_n319), .b(new_n316), .c(new_n320), .out0(new_n321));
  aoai13aa1n02x5               g226(.a(new_n317), .b(new_n291), .c(new_n192), .d(new_n280), .o1(new_n322));
  aoi012aa1n02x5               g227(.a(new_n316), .b(new_n322), .c(new_n320), .o1(new_n323));
  norp02aa1n02x5               g228(.a(new_n323), .b(new_n321), .o1(\s[31] ));
  xnrb03aa1n02x5               g229(.a(new_n121), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g230(.a(\a[3] ), .b(\b[2] ), .c(new_n121), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g232(.a(new_n123), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g233(.a(new_n123), .b(new_n110), .o1(new_n329));
  xobna2aa1n03x5               g234(.a(new_n106), .b(new_n329), .c(new_n109), .out0(\s[6] ));
  norp02aa1n02x5               g235(.a(new_n106), .b(new_n111), .o1(new_n331));
  aob012aa1n02x5               g236(.a(new_n125), .b(new_n123), .c(new_n331), .out0(new_n332));
  xorb03aa1n02x5               g237(.a(new_n332), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g238(.a(new_n101), .b(new_n332), .c(new_n102), .o1(new_n334));
  xnrb03aa1n02x5               g239(.a(new_n334), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g240(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


