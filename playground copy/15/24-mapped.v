// Benchmark "adder" written by ABC on Thu Jul 11 12:00:49 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n322, new_n323, new_n325, new_n327;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[2] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[1] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n02x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n02x5               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[6] ), .b(\b[5] ), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[5] ), .b(\b[4] ), .out0(new_n114));
  nano22aa1n02x4               g019(.a(new_n112), .b(new_n113), .c(new_n114), .out0(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  aoi112aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(new_n117), .b(new_n116), .o1(new_n118));
  160nm_fiao0012aa1n02p5x5     g023(.a(new_n108), .b(new_n110), .c(new_n109), .o(new_n119));
  oabi12aa1n02x5               g024(.a(new_n119), .b(new_n112), .c(new_n118), .out0(new_n120));
  aoi012aa1n02x5               g025(.a(new_n120), .b(new_n115), .c(new_n107), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .c(new_n121), .o1(new_n122));
  xorb03aa1n02x5               g027(.a(new_n122), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xnrc02aa1n02x5               g028(.a(\b[9] ), .b(\a[10] ), .out0(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  norp02aa1n02x5               g030(.a(new_n125), .b(new_n124), .o1(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n120), .c(new_n115), .d(new_n107), .o1(new_n127));
  160nm_ficinv00aa1n08x5       g032(.clk(\a[10] ), .clkout(new_n128));
  160nm_ficinv00aa1n08x5       g033(.clk(\b[9] ), .clkout(new_n129));
  norp02aa1n02x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  oaoi03aa1n02x5               g035(.a(new_n128), .b(new_n129), .c(new_n130), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n127), .c(new_n131), .out0(\s[11] ));
  nanp02aa1n02x5               g040(.a(new_n127), .b(new_n131), .o1(new_n136));
  aoi012aa1n02x5               g041(.a(new_n132), .b(new_n136), .c(new_n133), .o1(new_n137));
  xnrb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  160nm_ficinv00aa1n08x5       g043(.clk(\a[13] ), .clkout(new_n139));
  oao003aa1n02x5               g044(.a(new_n97), .b(new_n98), .c(new_n99), .carry(new_n140));
  nano23aa1n02x4               g045(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n141));
  aobi12aa1n02x5               g046(.a(new_n106), .b(new_n141), .c(new_n140), .out0(new_n142));
  nanb02aa1n02x5               g047(.a(new_n108), .b(new_n109), .out0(new_n143));
  nanb02aa1n02x5               g048(.a(new_n110), .b(new_n111), .out0(new_n144));
  nona23aa1n02x4               g049(.a(new_n113), .b(new_n114), .c(new_n144), .d(new_n143), .out0(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(new_n120), .clkout(new_n146));
  oai012aa1n02x5               g051(.a(new_n146), .b(new_n142), .c(new_n145), .o1(new_n147));
  norp02aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nano23aa1n02x4               g054(.a(new_n132), .b(new_n148), .c(new_n149), .d(new_n133), .out0(new_n150));
  nona23aa1n02x4               g055(.a(new_n149), .b(new_n133), .c(new_n132), .d(new_n148), .out0(new_n151));
  160nm_fiao0012aa1n02p5x5     g056(.a(new_n148), .b(new_n132), .c(new_n149), .o(new_n152));
  oabi12aa1n02x5               g057(.a(new_n152), .b(new_n151), .c(new_n131), .out0(new_n153));
  aoi013aa1n02x4               g058(.a(new_n153), .b(new_n147), .c(new_n126), .d(new_n150), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(new_n139), .out0(\s[13] ));
  oaoi03aa1n02x5               g060(.a(\a[13] ), .b(\b[12] ), .c(new_n154), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1n02x4               g066(.a(new_n158), .b(new_n160), .c(new_n161), .d(new_n159), .out0(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(\b[12] ), .clkout(new_n163));
  aoai13aa1n02x5               g068(.a(new_n161), .b(new_n160), .c(new_n139), .d(new_n163), .o1(new_n164));
  aobi12aa1n02x5               g069(.a(new_n164), .b(new_n153), .c(new_n162), .out0(new_n165));
  160nm_ficinv00aa1n08x5       g070(.clk(new_n165), .clkout(new_n166));
  nano32aa1n02x4               g071(.a(new_n121), .b(new_n162), .c(new_n126), .d(new_n150), .out0(new_n167));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  oabi12aa1n02x5               g075(.a(new_n170), .b(new_n167), .c(new_n166), .out0(new_n171));
  nano22aa1n02x4               g076(.a(new_n167), .b(new_n165), .c(new_n170), .out0(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n168), .clkout(new_n174));
  norp02aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n171), .c(new_n174), .out0(\s[16] ));
  nona23aa1n02x4               g083(.a(new_n176), .b(new_n169), .c(new_n168), .d(new_n175), .out0(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n126), .clkout(new_n180));
  nano23aa1n02x4               g085(.a(new_n168), .b(new_n175), .c(new_n176), .d(new_n169), .out0(new_n181));
  nano32aa1n02x4               g086(.a(new_n180), .b(new_n181), .c(new_n150), .d(new_n162), .out0(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n183));
  aoi012aa1n02x5               g088(.a(new_n175), .b(new_n168), .c(new_n176), .o1(new_n184));
  oai112aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n165), .d(new_n179), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[17] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\b[16] ), .clkout(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  nona23aa1n02x4               g096(.a(new_n162), .b(new_n126), .c(new_n179), .d(new_n151), .out0(new_n192));
  oaoi13aa1n02x5               g097(.a(new_n192), .b(new_n146), .c(new_n142), .d(new_n145), .o1(new_n193));
  oao003aa1n02x5               g098(.a(new_n128), .b(new_n129), .c(new_n130), .carry(new_n194));
  aoai13aa1n02x5               g099(.a(new_n162), .b(new_n152), .c(new_n150), .d(new_n194), .o1(new_n195));
  aoai13aa1n02x5               g100(.a(new_n184), .b(new_n179), .c(new_n195), .d(new_n164), .o1(new_n196));
  xroi22aa1d04x5               g101(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n197));
  oai012aa1n02x5               g102(.a(new_n197), .b(new_n196), .c(new_n193), .o1(new_n198));
  oai022aa1n02x5               g103(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n199));
  oaib12aa1n02x5               g104(.a(new_n199), .b(new_n187), .c(\b[17] ), .out0(new_n200));
  norp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n203), .clkout(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n198), .c(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n201), .clkout(new_n207));
  aoi012aa1n02x5               g112(.a(new_n203), .b(new_n198), .c(new_n200), .o1(new_n208));
  norp02aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanp02aa1n02x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  nano22aa1n02x4               g116(.a(new_n208), .b(new_n207), .c(new_n211), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n189), .b(new_n188), .o1(new_n213));
  oaoi03aa1n02x5               g118(.a(\a[18] ), .b(\b[17] ), .c(new_n213), .o1(new_n214));
  aoai13aa1n02x5               g119(.a(new_n204), .b(new_n214), .c(new_n185), .d(new_n197), .o1(new_n215));
  aoi012aa1n02x5               g120(.a(new_n211), .b(new_n215), .c(new_n207), .o1(new_n216));
  norp02aa1n02x5               g121(.a(new_n216), .b(new_n212), .o1(\s[20] ));
  nano23aa1n02x4               g122(.a(new_n201), .b(new_n209), .c(new_n210), .d(new_n202), .out0(new_n218));
  nanp02aa1n02x5               g123(.a(new_n197), .b(new_n218), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  oai012aa1n02x5               g125(.a(new_n220), .b(new_n196), .c(new_n193), .o1(new_n221));
  nona23aa1n02x4               g126(.a(new_n210), .b(new_n202), .c(new_n201), .d(new_n209), .out0(new_n222));
  aoi012aa1n02x5               g127(.a(new_n209), .b(new_n201), .c(new_n210), .o1(new_n223));
  oai012aa1n02x5               g128(.a(new_n223), .b(new_n222), .c(new_n200), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n224), .clkout(new_n225));
  xorc02aa1n02x5               g130(.a(\a[21] ), .b(\b[20] ), .out0(new_n226));
  xnbna2aa1n03x5               g131(.a(new_n226), .b(new_n221), .c(new_n225), .out0(\s[21] ));
  orn002aa1n02x5               g132(.a(\a[21] ), .b(\b[20] ), .o(new_n228));
  aobi12aa1n02x5               g133(.a(new_n226), .b(new_n221), .c(new_n225), .out0(new_n229));
  xnrc02aa1n02x5               g134(.a(\b[21] ), .b(\a[22] ), .out0(new_n230));
  nano22aa1n02x4               g135(.a(new_n229), .b(new_n228), .c(new_n230), .out0(new_n231));
  aoai13aa1n02x5               g136(.a(new_n226), .b(new_n224), .c(new_n185), .d(new_n220), .o1(new_n232));
  aoi012aa1n02x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .o1(new_n233));
  norp02aa1n02x5               g138(.a(new_n233), .b(new_n231), .o1(\s[22] ));
  nanp02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  nano22aa1n02x4               g140(.a(new_n230), .b(new_n228), .c(new_n235), .out0(new_n236));
  and003aa1n02x5               g141(.a(new_n197), .b(new_n236), .c(new_n218), .o(new_n237));
  oai012aa1n02x5               g142(.a(new_n237), .b(new_n196), .c(new_n193), .o1(new_n238));
  oaoi03aa1n02x5               g143(.a(\a[22] ), .b(\b[21] ), .c(new_n228), .o1(new_n239));
  aoi012aa1n02x5               g144(.a(new_n239), .b(new_n224), .c(new_n236), .o1(new_n240));
  xnrc02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .out0(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n241), .clkout(new_n242));
  xnbna2aa1n03x5               g147(.a(new_n242), .b(new_n238), .c(new_n240), .out0(\s[23] ));
  orn002aa1n02x5               g148(.a(\a[23] ), .b(\b[22] ), .o(new_n244));
  aoi012aa1n02x5               g149(.a(new_n241), .b(new_n238), .c(new_n240), .o1(new_n245));
  xnrc02aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .out0(new_n246));
  nano22aa1n02x4               g151(.a(new_n245), .b(new_n244), .c(new_n246), .out0(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n240), .clkout(new_n248));
  aoai13aa1n02x5               g153(.a(new_n242), .b(new_n248), .c(new_n185), .d(new_n237), .o1(new_n249));
  aoi012aa1n02x5               g154(.a(new_n246), .b(new_n249), .c(new_n244), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n250), .b(new_n247), .o1(\s[24] ));
  norp02aa1n02x5               g156(.a(new_n246), .b(new_n241), .o1(new_n252));
  nano22aa1n02x4               g157(.a(new_n219), .b(new_n236), .c(new_n252), .out0(new_n253));
  oai012aa1n02x5               g158(.a(new_n253), .b(new_n196), .c(new_n193), .o1(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n223), .clkout(new_n255));
  aoai13aa1n02x5               g160(.a(new_n236), .b(new_n255), .c(new_n218), .d(new_n214), .o1(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n239), .clkout(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n252), .clkout(new_n258));
  oaoi03aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .c(new_n244), .o1(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n259), .clkout(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n258), .c(new_n256), .d(new_n257), .o1(new_n261));
  xnrc02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .out0(new_n262));
  aoib12aa1n02x5               g167(.a(new_n262), .b(new_n254), .c(new_n261), .out0(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n262), .clkout(new_n264));
  aoi112aa1n02x5               g169(.a(new_n264), .b(new_n261), .c(new_n185), .d(new_n253), .o1(new_n265));
  norp02aa1n02x5               g170(.a(new_n263), .b(new_n265), .o1(\s[25] ));
  norp02aa1n02x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n267), .clkout(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[25] ), .b(\a[26] ), .out0(new_n269));
  nano22aa1n02x4               g174(.a(new_n263), .b(new_n268), .c(new_n269), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n264), .b(new_n261), .c(new_n185), .d(new_n253), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n269), .b(new_n271), .c(new_n268), .o1(new_n272));
  norp02aa1n02x5               g177(.a(new_n272), .b(new_n270), .o1(\s[26] ));
  norp02aa1n02x5               g178(.a(new_n269), .b(new_n262), .o1(new_n274));
  nano32aa1n02x4               g179(.a(new_n219), .b(new_n274), .c(new_n236), .d(new_n252), .out0(new_n275));
  oai012aa1n02x5               g180(.a(new_n275), .b(new_n196), .c(new_n193), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n277));
  aobi12aa1n02x5               g182(.a(new_n277), .b(new_n261), .c(new_n274), .out0(new_n278));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  norb02aa1n02x5               g185(.a(new_n280), .b(new_n279), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n276), .c(new_n278), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n279), .clkout(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  aoai13aa1n02x5               g189(.a(new_n252), .b(new_n239), .c(new_n224), .d(new_n236), .o1(new_n285));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n274), .clkout(new_n286));
  aoai13aa1n02x5               g191(.a(new_n277), .b(new_n286), .c(new_n285), .d(new_n260), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n280), .b(new_n287), .c(new_n185), .d(new_n275), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n284), .b(new_n288), .c(new_n283), .o1(new_n289));
  aobi12aa1n02x5               g194(.a(new_n280), .b(new_n276), .c(new_n278), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n283), .c(new_n284), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(\s[28] ));
  nano22aa1n02x4               g197(.a(new_n284), .b(new_n283), .c(new_n280), .out0(new_n293));
  aoai13aa1n02x5               g198(.a(new_n293), .b(new_n287), .c(new_n185), .d(new_n275), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[28] ), .b(\a[29] ), .out0(new_n296));
  aoi012aa1n02x5               g201(.a(new_n296), .b(new_n294), .c(new_n295), .o1(new_n297));
  aobi12aa1n02x5               g202(.a(new_n293), .b(new_n276), .c(new_n278), .out0(new_n298));
  nano22aa1n02x4               g203(.a(new_n298), .b(new_n295), .c(new_n296), .out0(new_n299));
  norp02aa1n02x5               g204(.a(new_n297), .b(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g206(.a(new_n281), .b(new_n296), .c(new_n284), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n302), .b(new_n287), .c(new_n185), .d(new_n275), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  aoi012aa1n02x5               g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  aobi12aa1n02x5               g211(.a(new_n302), .b(new_n276), .c(new_n278), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n307), .b(new_n304), .c(new_n305), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n306), .b(new_n308), .o1(\s[30] ));
  norb03aa1n02x5               g214(.a(new_n293), .b(new_n305), .c(new_n296), .out0(new_n310));
  aobi12aa1n02x5               g215(.a(new_n310), .b(new_n276), .c(new_n278), .out0(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  nano22aa1n02x4               g218(.a(new_n311), .b(new_n312), .c(new_n313), .out0(new_n314));
  aoai13aa1n02x5               g219(.a(new_n310), .b(new_n287), .c(new_n185), .d(new_n275), .o1(new_n315));
  aoi012aa1n02x5               g220(.a(new_n313), .b(new_n315), .c(new_n312), .o1(new_n316));
  norp02aa1n02x5               g221(.a(new_n316), .b(new_n314), .o1(\s[31] ));
  xnrb03aa1n02x5               g222(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g223(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g225(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai112aa1n02x5               g226(.a(new_n106), .b(new_n114), .c(new_n105), .d(new_n100), .o1(new_n322));
  aob012aa1n02x5               g227(.a(new_n322), .b(\b[4] ), .c(\a[5] ), .out0(new_n323));
  xnrc02aa1n02x5               g228(.a(new_n323), .b(new_n113), .out0(\s[6] ));
  oao003aa1n02x5               g229(.a(\a[6] ), .b(\b[5] ), .c(new_n323), .carry(new_n325));
  xnrb03aa1n02x5               g230(.a(new_n325), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g231(.a(\a[7] ), .b(\b[6] ), .c(new_n325), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g233(.a(new_n147), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


