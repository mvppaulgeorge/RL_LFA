// Benchmark "adder" written by ABC on Wed Jul 10 16:48:07 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n323, new_n325, new_n326, new_n328;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  xnrc02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .out0(new_n103));
  160nm_ficinv00aa1n08x5       g008(.clk(\a[3] ), .clkout(new_n104));
  nanb02aa1n02x5               g009(.a(\b[2] ), .b(new_n104), .out0(new_n105));
  oao003aa1n02x5               g010(.a(\a[4] ), .b(\b[3] ), .c(new_n105), .carry(new_n106));
  oai013aa1n02x4               g011(.a(new_n106), .b(new_n102), .c(new_n103), .d(new_n101), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  norb02aa1n02x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norb02aa1n02x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  nano23aa1n02x4               g020(.a(new_n115), .b(new_n114), .c(new_n113), .d(new_n110), .out0(new_n116));
  aoi112aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(new_n109), .clkout(new_n118));
  xorc02aa1n02x5               g023(.a(\a[7] ), .b(\b[6] ), .out0(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\a[5] ), .clkout(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[4] ), .clkout(new_n121));
  aoi012aa1n02x5               g026(.a(new_n108), .b(new_n120), .c(new_n121), .o1(new_n122));
  nona23aa1n02x4               g027(.a(new_n119), .b(new_n113), .c(new_n122), .d(new_n118), .out0(new_n123));
  nona22aa1n02x4               g028(.a(new_n123), .b(new_n117), .c(new_n111), .out0(new_n124));
  xorc02aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(new_n107), .d(new_n116), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[10] ), .b(\b[9] ), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n97), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g033(.clk(\b[9] ), .clkout(new_n129));
  nanb02aa1n02x5               g034(.a(\a[10] ), .b(new_n129), .out0(new_n130));
  and002aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o(new_n131));
  aoi013aa1n02x4               g036(.a(new_n131), .b(new_n126), .c(new_n130), .d(new_n97), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n136), .clkout(new_n137));
  aoi113aa1n02x5               g042(.a(new_n137), .b(new_n131), .c(new_n126), .d(new_n130), .e(new_n97), .o1(new_n138));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  norp03aa1n02x5               g046(.a(new_n138), .b(new_n141), .c(new_n134), .o1(new_n142));
  aoai13aa1n02x5               g047(.a(new_n141), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(\s[12] ));
  nanp02aa1n02x5               g049(.a(new_n107), .b(new_n116), .o1(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(new_n122), .clkout(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n111), .clkout(new_n147));
  nano32aa1n02x4               g052(.a(new_n114), .b(new_n147), .c(new_n112), .d(new_n109), .out0(new_n148));
  aoi112aa1n02x5               g053(.a(new_n111), .b(new_n117), .c(new_n148), .d(new_n146), .o1(new_n149));
  nano23aa1n02x4               g054(.a(new_n134), .b(new_n139), .c(new_n140), .d(new_n135), .out0(new_n150));
  nanp03aa1n02x5               g055(.a(new_n150), .b(new_n125), .c(new_n127), .o1(new_n151));
  oai022aa1n02x5               g056(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n152));
  oaib12aa1n02x5               g057(.a(new_n152), .b(new_n129), .c(\a[10] ), .out0(new_n153));
  nona23aa1n02x4               g058(.a(new_n140), .b(new_n135), .c(new_n134), .d(new_n139), .out0(new_n154));
  160nm_fiao0012aa1n02p5x5     g059(.a(new_n139), .b(new_n134), .c(new_n140), .o(new_n155));
  oabi12aa1n02x5               g060(.a(new_n155), .b(new_n154), .c(new_n153), .out0(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n156), .clkout(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n151), .c(new_n145), .d(new_n149), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g064(.clk(\a[14] ), .clkout(new_n160));
  norp02aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  xnrc02aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .out0(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  aoi012aa1n02x5               g068(.a(new_n161), .b(new_n158), .c(new_n163), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(new_n160), .out0(\s[14] ));
  xnrc02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .out0(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n166), .clkout(new_n167));
  xnrc02aa1n02x5               g072(.a(\b[13] ), .b(\a[14] ), .out0(new_n168));
  norp02aa1n02x5               g073(.a(new_n168), .b(new_n162), .o1(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(\b[13] ), .clkout(new_n170));
  oao003aa1n02x5               g075(.a(new_n160), .b(new_n170), .c(new_n161), .carry(new_n171));
  aoai13aa1n02x5               g076(.a(new_n167), .b(new_n171), .c(new_n158), .d(new_n169), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n167), .b(new_n171), .c(new_n158), .d(new_n169), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  xnrc02aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .out0(new_n175));
  oai112aa1n02x5               g080(.a(new_n172), .b(new_n175), .c(\b[14] ), .d(\a[15] ), .o1(new_n176));
  oaoi13aa1n02x5               g081(.a(new_n175), .b(new_n172), .c(\a[15] ), .d(\b[14] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(\s[16] ));
  norp02aa1n02x5               g083(.a(new_n175), .b(new_n166), .o1(new_n179));
  nano22aa1n02x4               g084(.a(new_n151), .b(new_n169), .c(new_n179), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n124), .c(new_n107), .d(new_n116), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n179), .b(new_n171), .c(new_n156), .d(new_n169), .o1(new_n182));
  aoi112aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n183));
  oab012aa1n02x4               g088(.a(new_n183), .b(\a[16] ), .c(\b[15] ), .out0(new_n184));
  nanp03aa1n02x5               g089(.a(new_n181), .b(new_n182), .c(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[17] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\b[16] ), .clkout(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  nanp02aa1n02x5               g096(.a(new_n145), .b(new_n149), .o1(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n171), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n179), .clkout(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(new_n153), .clkout(new_n195));
  aoai13aa1n02x5               g100(.a(new_n169), .b(new_n155), .c(new_n150), .d(new_n195), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n184), .b(new_n194), .c(new_n196), .d(new_n193), .o1(new_n197));
  xroi22aa1d04x5               g102(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n197), .c(new_n192), .d(new_n180), .o1(new_n199));
  oai022aa1n02x5               g104(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n200));
  oaib12aa1n02x5               g105(.a(new_n200), .b(new_n187), .c(\b[17] ), .out0(new_n201));
  norp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n204), .clkout(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n199), .c(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g112(.clk(new_n202), .clkout(new_n208));
  aoi012aa1n02x5               g113(.a(new_n204), .b(new_n199), .c(new_n201), .o1(new_n209));
  norp02aa1n02x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  nano22aa1n02x4               g117(.a(new_n209), .b(new_n208), .c(new_n212), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n189), .b(new_n188), .o1(new_n214));
  oaoi03aa1n02x5               g119(.a(\a[18] ), .b(\b[17] ), .c(new_n214), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n205), .b(new_n215), .c(new_n185), .d(new_n198), .o1(new_n216));
  aoi012aa1n02x5               g121(.a(new_n212), .b(new_n216), .c(new_n208), .o1(new_n217));
  norp02aa1n02x5               g122(.a(new_n217), .b(new_n213), .o1(\s[20] ));
  nano23aa1n02x4               g123(.a(new_n202), .b(new_n210), .c(new_n211), .d(new_n203), .out0(new_n219));
  nanp02aa1n02x5               g124(.a(new_n198), .b(new_n219), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n220), .clkout(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n197), .c(new_n192), .d(new_n180), .o1(new_n222));
  nona23aa1n02x4               g127(.a(new_n211), .b(new_n203), .c(new_n202), .d(new_n210), .out0(new_n223));
  aoi012aa1n02x5               g128(.a(new_n210), .b(new_n202), .c(new_n211), .o1(new_n224));
  oai012aa1n02x5               g129(.a(new_n224), .b(new_n223), .c(new_n201), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  norp02aa1n02x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  xnbna2aa1n03x5               g134(.a(new_n229), .b(new_n222), .c(new_n226), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n227), .clkout(new_n231));
  aobi12aa1n02x5               g136(.a(new_n229), .b(new_n222), .c(new_n226), .out0(new_n232));
  xnrc02aa1n02x5               g137(.a(\b[21] ), .b(\a[22] ), .out0(new_n233));
  nano22aa1n02x4               g138(.a(new_n232), .b(new_n231), .c(new_n233), .out0(new_n234));
  aoai13aa1n02x5               g139(.a(new_n229), .b(new_n225), .c(new_n185), .d(new_n221), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .o1(new_n236));
  norp02aa1n02x5               g141(.a(new_n236), .b(new_n234), .o1(\s[22] ));
  nano22aa1n02x4               g142(.a(new_n233), .b(new_n231), .c(new_n228), .out0(new_n238));
  and003aa1n02x5               g143(.a(new_n198), .b(new_n238), .c(new_n219), .o(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n197), .c(new_n192), .d(new_n180), .o1(new_n240));
  oao003aa1n02x5               g145(.a(\a[22] ), .b(\b[21] ), .c(new_n231), .carry(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n241), .clkout(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n225), .c(new_n238), .o1(new_n243));
  xnrc02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  xnbna2aa1n03x5               g150(.a(new_n245), .b(new_n240), .c(new_n243), .out0(\s[23] ));
  norp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  aoi012aa1n02x5               g153(.a(new_n244), .b(new_n240), .c(new_n243), .o1(new_n249));
  xnrc02aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .out0(new_n250));
  nano22aa1n02x4               g155(.a(new_n249), .b(new_n248), .c(new_n250), .out0(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n243), .clkout(new_n252));
  aoai13aa1n02x5               g157(.a(new_n245), .b(new_n252), .c(new_n185), .d(new_n239), .o1(new_n253));
  aoi012aa1n02x5               g158(.a(new_n250), .b(new_n253), .c(new_n248), .o1(new_n254));
  norp02aa1n02x5               g159(.a(new_n254), .b(new_n251), .o1(\s[24] ));
  norp02aa1n02x5               g160(.a(new_n250), .b(new_n244), .o1(new_n256));
  nano22aa1n02x4               g161(.a(new_n220), .b(new_n238), .c(new_n256), .out0(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n197), .c(new_n192), .d(new_n180), .o1(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n224), .clkout(new_n259));
  aoai13aa1n02x5               g164(.a(new_n238), .b(new_n259), .c(new_n219), .d(new_n215), .o1(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n256), .clkout(new_n261));
  oao003aa1n02x5               g166(.a(\a[24] ), .b(\b[23] ), .c(new_n248), .carry(new_n262));
  aoai13aa1n02x5               g167(.a(new_n262), .b(new_n261), .c(new_n260), .d(new_n241), .o1(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .out0(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n265), .clkout(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n258), .c(new_n264), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n268), .clkout(new_n269));
  aoi012aa1n02x5               g174(.a(new_n265), .b(new_n258), .c(new_n264), .o1(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[25] ), .b(\a[26] ), .out0(new_n271));
  nano22aa1n02x4               g176(.a(new_n270), .b(new_n269), .c(new_n271), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n266), .b(new_n263), .c(new_n185), .d(new_n257), .o1(new_n273));
  aoi012aa1n02x5               g178(.a(new_n271), .b(new_n273), .c(new_n269), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n274), .b(new_n272), .o1(\s[26] ));
  norp02aa1n02x5               g180(.a(new_n271), .b(new_n265), .o1(new_n276));
  nano32aa1n02x4               g181(.a(new_n220), .b(new_n276), .c(new_n238), .d(new_n256), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n197), .c(new_n192), .d(new_n180), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n279));
  aobi12aa1n02x5               g184(.a(new_n279), .b(new_n263), .c(new_n276), .out0(new_n280));
  xorc02aa1n02x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n280), .out0(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n283), .clkout(new_n284));
  aobi12aa1n02x5               g189(.a(new_n281), .b(new_n278), .c(new_n280), .out0(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[27] ), .b(\a[28] ), .out0(new_n286));
  nano22aa1n02x4               g191(.a(new_n285), .b(new_n284), .c(new_n286), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n256), .b(new_n242), .c(new_n225), .d(new_n238), .o1(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(new_n276), .clkout(new_n289));
  aoai13aa1n02x5               g194(.a(new_n279), .b(new_n289), .c(new_n288), .d(new_n262), .o1(new_n290));
  aoai13aa1n02x5               g195(.a(new_n281), .b(new_n290), .c(new_n185), .d(new_n277), .o1(new_n291));
  aoi012aa1n02x5               g196(.a(new_n286), .b(new_n291), .c(new_n284), .o1(new_n292));
  norp02aa1n02x5               g197(.a(new_n292), .b(new_n287), .o1(\s[28] ));
  norb02aa1n02x5               g198(.a(new_n281), .b(new_n286), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n185), .d(new_n277), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x5               g203(.a(new_n294), .b(new_n278), .c(new_n280), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g207(.a(new_n281), .b(new_n297), .c(new_n286), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n290), .c(new_n185), .d(new_n277), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  aoi012aa1n02x5               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n02x5               g212(.a(new_n303), .b(new_n278), .c(new_n280), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n02x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  norb02aa1n02x5               g215(.a(new_n303), .b(new_n306), .out0(new_n311));
  aobi12aa1n02x5               g216(.a(new_n311), .b(new_n278), .c(new_n280), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[30] ), .b(\a[31] ), .out0(new_n314));
  nano22aa1n02x4               g219(.a(new_n312), .b(new_n313), .c(new_n314), .out0(new_n315));
  aoai13aa1n02x5               g220(.a(new_n311), .b(new_n290), .c(new_n185), .d(new_n277), .o1(new_n316));
  aoi012aa1n02x5               g221(.a(new_n314), .b(new_n316), .c(new_n313), .o1(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  xorb03aa1n02x5               g223(.a(new_n101), .b(\b[2] ), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g224(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n02x5               g227(.a(new_n120), .b(new_n121), .c(new_n107), .carry(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g229(.a(new_n119), .b(new_n108), .c(new_n323), .d(new_n109), .o1(new_n325));
  aoi112aa1n02x5               g230(.a(new_n119), .b(new_n108), .c(new_n323), .d(new_n109), .o1(new_n326));
  norb02aa1n02x5               g231(.a(new_n325), .b(new_n326), .out0(\s[7] ));
  orn002aa1n02x5               g232(.a(\a[7] ), .b(\b[6] ), .o(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n113), .b(new_n325), .c(new_n328), .out0(\s[8] ));
  xnbna2aa1n03x5               g234(.a(new_n125), .b(new_n145), .c(new_n149), .out0(\s[9] ));
endmodule


