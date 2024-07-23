// Benchmark "adder" written by ABC on Thu Jul 11 12:14:21 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n332, new_n335, new_n336, new_n338, new_n340;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(new_n100), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oai012aa1n02x5               g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  xnrc02aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .out0(new_n106));
  norp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanb02aa1n02x5               g013(.a(new_n107), .b(new_n108), .out0(new_n109));
  aoi112aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n110));
  oab012aa1n02x4               g015(.a(new_n110), .b(\a[4] ), .c(\b[3] ), .out0(new_n111));
  oai013aa1n02x4               g016(.a(new_n111), .b(new_n106), .c(new_n105), .d(new_n109), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  norp03aa1n02x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\a[6] ), .clkout(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\b[5] ), .clkout(new_n122));
  norp02aa1n02x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(new_n121), .b(new_n122), .c(new_n123), .o1(new_n124));
  160nm_fiao0012aa1n02p5x5     g029(.a(new_n113), .b(new_n115), .c(new_n114), .o(new_n125));
  oabi12aa1n02x5               g030(.a(new_n125), .b(new_n117), .c(new_n124), .out0(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n100), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n129));
  xobna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  nanp02aa1n02x5               g035(.a(new_n129), .b(new_n101), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n97), .c(new_n131), .d(new_n98), .o1(new_n135));
  aoi112aa1n02x5               g040(.a(new_n134), .b(new_n97), .c(new_n131), .d(new_n98), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  norp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  nona22aa1n02x4               g045(.a(new_n135), .b(new_n140), .c(new_n132), .out0(new_n141));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n132), .clkout(new_n142));
  160nm_ficinv00aa1n08x5       g047(.clk(new_n140), .clkout(new_n143));
  aoi012aa1n02x5               g048(.a(new_n143), .b(new_n135), .c(new_n142), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n141), .b(new_n144), .out0(\s[12] ));
  nano23aa1n02x4               g050(.a(new_n132), .b(new_n138), .c(new_n139), .d(new_n133), .out0(new_n146));
  nano23aa1n02x4               g051(.a(new_n97), .b(new_n100), .c(new_n127), .d(new_n98), .out0(new_n147));
  nanp02aa1n02x5               g052(.a(new_n147), .b(new_n146), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n150));
  nona23aa1n02x4               g055(.a(new_n139), .b(new_n133), .c(new_n132), .d(new_n138), .out0(new_n151));
  oai012aa1n02x5               g056(.a(new_n98), .b(new_n100), .c(new_n97), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n138), .b(new_n132), .c(new_n139), .o1(new_n153));
  oai012aa1n02x5               g058(.a(new_n153), .b(new_n151), .c(new_n152), .o1(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  norp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n150), .c(new_n155), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n156), .clkout(new_n160));
  xorc02aa1n02x5               g065(.a(\a[4] ), .b(\b[3] ), .out0(new_n161));
  norb02aa1n02x5               g066(.a(new_n108), .b(new_n107), .out0(new_n162));
  nanb03aa1n02x5               g067(.a(new_n105), .b(new_n161), .c(new_n162), .out0(new_n163));
  nano23aa1n02x4               g068(.a(new_n113), .b(new_n115), .c(new_n116), .d(new_n114), .out0(new_n164));
  nona22aa1n02x4               g069(.a(new_n164), .b(new_n118), .c(new_n119), .out0(new_n165));
  oao003aa1n02x5               g070(.a(new_n121), .b(new_n122), .c(new_n123), .carry(new_n166));
  aoi012aa1n02x5               g071(.a(new_n125), .b(new_n164), .c(new_n166), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n165), .c(new_n163), .d(new_n111), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n158), .b(new_n154), .c(new_n168), .d(new_n149), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n169), .c(new_n160), .out0(\s[14] ));
  nona23aa1n02x4               g078(.a(new_n171), .b(new_n157), .c(new_n156), .d(new_n170), .out0(new_n174));
  aoi012aa1n02x5               g079(.a(new_n170), .b(new_n156), .c(new_n171), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n174), .c(new_n150), .d(new_n155), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nanp02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norp02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(new_n180), .clkout(new_n181));
  nanp02aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  aoi122aa1n02x5               g087(.a(new_n178), .b(new_n182), .c(new_n181), .d(new_n176), .e(new_n179), .o1(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(new_n178), .clkout(new_n184));
  nanb02aa1n02x5               g089(.a(new_n178), .b(new_n179), .out0(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n185), .clkout(new_n186));
  nanp02aa1n02x5               g091(.a(new_n176), .b(new_n186), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(new_n180), .b(new_n182), .out0(new_n188));
  aoi012aa1n02x5               g093(.a(new_n188), .b(new_n187), .c(new_n184), .o1(new_n189));
  norp02aa1n02x5               g094(.a(new_n189), .b(new_n183), .o1(\s[16] ));
  nano23aa1n02x4               g095(.a(new_n178), .b(new_n180), .c(new_n182), .d(new_n179), .out0(new_n191));
  nano32aa1n02x4               g096(.a(new_n148), .b(new_n191), .c(new_n158), .d(new_n172), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n126), .c(new_n112), .d(new_n120), .o1(new_n193));
  norp03aa1n02x5               g098(.a(new_n174), .b(new_n188), .c(new_n185), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n195));
  oai013aa1n02x4               g100(.a(new_n181), .b(new_n175), .c(new_n185), .d(new_n188), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n196), .b(new_n195), .c(new_n154), .d(new_n194), .o1(new_n197));
  norp02aa1n02x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  xobna2aa1n03x5               g105(.a(new_n200), .b(new_n193), .c(new_n197), .out0(\s[17] ));
  nanb03aa1n02x5               g106(.a(new_n200), .b(new_n193), .c(new_n197), .out0(new_n202));
  xorc02aa1n02x5               g107(.a(\a[18] ), .b(\b[17] ), .out0(new_n203));
  xobna2aa1n03x5               g108(.a(new_n203), .b(new_n202), .c(new_n199), .out0(\s[18] ));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n200), .out0(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(new_n205), .clkout(new_n206));
  aoi112aa1n02x5               g111(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n207));
  oabi12aa1n02x5               g112(.a(new_n207), .b(\a[18] ), .c(\b[17] ), .out0(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n206), .c(new_n193), .d(new_n197), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  norp02aa1n02x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  aoi112aa1n02x5               g122(.a(new_n213), .b(new_n217), .c(new_n210), .d(new_n214), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n217), .b(new_n213), .c(new_n210), .d(new_n214), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(\s[20] ));
  nano23aa1n02x4               g125(.a(new_n213), .b(new_n215), .c(new_n216), .d(new_n214), .out0(new_n221));
  nanb03aa1n02x5               g126(.a(new_n200), .b(new_n221), .c(new_n203), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n223));
  norp02aa1n02x5               g128(.a(\b[17] ), .b(\a[18] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n214), .b(new_n213), .out0(new_n225));
  oai112aa1n02x5               g130(.a(new_n225), .b(new_n217), .c(new_n207), .d(new_n224), .o1(new_n226));
  nona22aa1n02x4               g131(.a(new_n226), .b(new_n223), .c(new_n215), .out0(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n227), .clkout(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n222), .c(new_n193), .d(new_n197), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nanp02aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  norp02aa1n02x5               g137(.a(\b[21] ), .b(\a[22] ), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n234), .b(new_n233), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n229), .d(new_n232), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[22] ));
  nona23aa1n02x4               g143(.a(new_n234), .b(new_n232), .c(new_n231), .d(new_n233), .out0(new_n239));
  nona23aa1n02x4               g144(.a(new_n221), .b(new_n203), .c(new_n239), .d(new_n200), .out0(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n239), .clkout(new_n241));
  aoi012aa1n02x5               g146(.a(new_n233), .b(new_n231), .c(new_n234), .o1(new_n242));
  aobi12aa1n02x5               g147(.a(new_n242), .b(new_n227), .c(new_n241), .out0(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n240), .c(new_n193), .d(new_n197), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  norp02aa1n02x5               g152(.a(\b[23] ), .b(\a[24] ), .o1(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  nanp02aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  aoi122aa1n02x5               g155(.a(new_n246), .b(new_n249), .c(new_n250), .d(new_n244), .e(new_n247), .o1(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n246), .clkout(new_n252));
  nanb02aa1n02x5               g157(.a(new_n246), .b(new_n247), .out0(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n253), .clkout(new_n254));
  nanp02aa1n02x5               g159(.a(new_n244), .b(new_n254), .o1(new_n255));
  nanb02aa1n02x5               g160(.a(new_n248), .b(new_n250), .out0(new_n256));
  aoi012aa1n02x5               g161(.a(new_n256), .b(new_n255), .c(new_n252), .o1(new_n257));
  norp02aa1n02x5               g162(.a(new_n257), .b(new_n251), .o1(\s[24] ));
  nona23aa1n02x4               g163(.a(new_n250), .b(new_n247), .c(new_n246), .d(new_n248), .out0(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n259), .clkout(new_n260));
  nanb03aa1n02x5               g165(.a(new_n222), .b(new_n260), .c(new_n241), .out0(new_n261));
  norp02aa1n02x5               g166(.a(new_n259), .b(new_n239), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(new_n246), .b(new_n250), .o1(new_n263));
  oai112aa1n02x5               g168(.a(new_n263), .b(new_n249), .c(new_n259), .d(new_n242), .o1(new_n264));
  aoi012aa1n02x5               g169(.a(new_n264), .b(new_n227), .c(new_n262), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n261), .c(new_n193), .d(new_n197), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  xorc02aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xorc02aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  aoi112aa1n02x5               g175(.a(new_n268), .b(new_n270), .c(new_n266), .d(new_n269), .o1(new_n271));
  aoai13aa1n02x5               g176(.a(new_n270), .b(new_n268), .c(new_n266), .d(new_n269), .o1(new_n272));
  norb02aa1n02x5               g177(.a(new_n272), .b(new_n271), .out0(\s[26] ));
  nanp02aa1n02x5               g178(.a(new_n154), .b(new_n194), .o1(new_n274));
  nona22aa1n02x4               g179(.a(new_n274), .b(new_n196), .c(new_n195), .out0(new_n275));
  and002aa1n02x5               g180(.a(new_n270), .b(new_n269), .o(new_n276));
  nano22aa1n02x4               g181(.a(new_n240), .b(new_n260), .c(new_n276), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n275), .c(new_n168), .d(new_n192), .o1(new_n278));
  norp02aa1n02x5               g183(.a(\b[25] ), .b(\a[26] ), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n279), .clkout(new_n280));
  aoi112aa1n02x5               g185(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n281), .clkout(new_n282));
  nanp02aa1n02x5               g187(.a(new_n227), .b(new_n262), .o1(new_n283));
  norp03aa1n02x5               g188(.a(new_n242), .b(new_n253), .c(new_n256), .o1(new_n284));
  nano22aa1n02x4               g189(.a(new_n284), .b(new_n249), .c(new_n263), .out0(new_n285));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n276), .clkout(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n283), .c(new_n285), .o1(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n280), .c(new_n282), .out0(new_n288));
  xorc02aa1n02x5               g193(.a(\a[27] ), .b(\b[26] ), .out0(new_n289));
  xnbna2aa1n03x5               g194(.a(new_n289), .b(new_n288), .c(new_n278), .out0(\s[27] ));
  norp02aa1n02x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n291), .clkout(new_n292));
  nanp02aa1n02x5               g197(.a(new_n193), .b(new_n197), .o1(new_n293));
  aoai13aa1n02x5               g198(.a(new_n276), .b(new_n264), .c(new_n227), .d(new_n262), .o1(new_n294));
  nona22aa1n02x4               g199(.a(new_n294), .b(new_n281), .c(new_n279), .out0(new_n295));
  aoai13aa1n02x5               g200(.a(new_n289), .b(new_n295), .c(new_n293), .d(new_n277), .o1(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[27] ), .b(\a[28] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n296), .c(new_n292), .o1(new_n298));
  160nm_ficinv00aa1n08x5       g203(.clk(new_n289), .clkout(new_n299));
  aoi012aa1n02x5               g204(.a(new_n299), .b(new_n288), .c(new_n278), .o1(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n292), .c(new_n297), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n298), .b(new_n301), .o1(\s[28] ));
  norb02aa1n02x5               g207(.a(new_n289), .b(new_n297), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n295), .c(new_n293), .d(new_n277), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[28] ), .b(\b[27] ), .c(new_n292), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[28] ), .b(\a[29] ), .out0(new_n306));
  aoi012aa1n02x5               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  160nm_ficinv00aa1n08x5       g212(.clk(new_n303), .clkout(new_n308));
  aoi012aa1n02x5               g213(.a(new_n308), .b(new_n288), .c(new_n278), .o1(new_n309));
  nano22aa1n02x4               g214(.a(new_n309), .b(new_n305), .c(new_n306), .out0(new_n310));
  norp02aa1n02x5               g215(.a(new_n307), .b(new_n310), .o1(\s[29] ));
  xorb03aa1n02x5               g216(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g217(.a(new_n289), .b(new_n306), .c(new_n297), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n295), .c(new_n293), .d(new_n277), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .c(new_n305), .carry(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[29] ), .b(\a[30] ), .out0(new_n316));
  aoi012aa1n02x5               g221(.a(new_n316), .b(new_n314), .c(new_n315), .o1(new_n317));
  160nm_ficinv00aa1n08x5       g222(.clk(new_n313), .clkout(new_n318));
  aoi012aa1n02x5               g223(.a(new_n318), .b(new_n288), .c(new_n278), .o1(new_n319));
  nano22aa1n02x4               g224(.a(new_n319), .b(new_n315), .c(new_n316), .out0(new_n320));
  norp02aa1n02x5               g225(.a(new_n317), .b(new_n320), .o1(\s[30] ));
  norb02aa1n02x5               g226(.a(new_n313), .b(new_n316), .out0(new_n322));
  160nm_ficinv00aa1n08x5       g227(.clk(new_n322), .clkout(new_n323));
  aoi012aa1n02x5               g228(.a(new_n323), .b(new_n288), .c(new_n278), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .c(new_n315), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[30] ), .b(\a[31] ), .out0(new_n326));
  nano22aa1n02x4               g231(.a(new_n324), .b(new_n325), .c(new_n326), .out0(new_n327));
  aoai13aa1n02x5               g232(.a(new_n322), .b(new_n295), .c(new_n293), .d(new_n277), .o1(new_n328));
  aoi012aa1n02x5               g233(.a(new_n326), .b(new_n328), .c(new_n325), .o1(new_n329));
  norp02aa1n02x5               g234(.a(new_n329), .b(new_n327), .o1(\s[31] ));
  xnrb03aa1n02x5               g235(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g236(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n332));
  xorb03aa1n02x5               g237(.a(new_n332), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g238(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g239(.a(\b[4] ), .b(\a[5] ), .o1(new_n335));
  oai012aa1n02x5               g240(.a(new_n335), .b(new_n112), .c(new_n123), .o1(new_n336));
  xorb03aa1n02x5               g241(.a(new_n336), .b(\b[5] ), .c(new_n121), .out0(\s[6] ));
  oai012aa1n02x5               g242(.a(new_n124), .b(new_n336), .c(new_n118), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g244(.a(new_n115), .b(new_n338), .c(new_n116), .o1(new_n340));
  xnrb03aa1n02x5               g245(.a(new_n340), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g246(.a(new_n168), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


