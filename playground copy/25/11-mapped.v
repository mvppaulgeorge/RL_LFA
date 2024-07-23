// Benchmark "adder" written by ABC on Thu Jul 11 12:43:23 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n343, new_n346, new_n347,
    new_n349, new_n351;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(new_n99), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aob012aa1n02x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  xorc02aa1n02x5               g008(.a(\a[4] ), .b(\b[3] ), .out0(new_n104));
  xorc02aa1n02x5               g009(.a(\a[3] ), .b(\b[2] ), .out0(new_n105));
  nanp03aa1n02x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  160nm_ficinv00aa1n08x5       g011(.clk(\a[3] ), .clkout(new_n107));
  160nm_ficinv00aa1n08x5       g012(.clk(\b[2] ), .clkout(new_n108));
  nanp02aa1n02x5               g013(.a(new_n108), .b(new_n107), .o1(new_n109));
  oaoi03aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(new_n109), .o1(new_n110));
  160nm_ficinv00aa1n08x5       g015(.clk(new_n110), .clkout(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n02x4               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  nona22aa1n02x4               g023(.a(new_n116), .b(new_n117), .c(new_n118), .out0(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\a[6] ), .clkout(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[5] ), .clkout(new_n121));
  norp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oao003aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .carry(new_n123));
  160nm_fiao0012aa1n02p5x5     g028(.a(new_n112), .b(new_n114), .c(new_n113), .o(new_n124));
  aoi012aa1n02x5               g029(.a(new_n124), .b(new_n116), .c(new_n123), .o1(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n119), .c(new_n106), .d(new_n111), .o1(new_n126));
  oaoi03aa1n02x5               g031(.a(new_n97), .b(new_n98), .c(new_n126), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  nanp02aa1n02x5               g036(.a(new_n98), .b(new_n97), .o1(new_n132));
  oaoi03aa1n02x5               g037(.a(\a[10] ), .b(\b[9] ), .c(new_n132), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[8] ), .b(\a[9] ), .o1(new_n134));
  norp02aa1n02x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  nano22aa1n02x4               g042(.a(new_n137), .b(new_n132), .c(new_n134), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n131), .b(new_n133), .c(new_n126), .d(new_n138), .o1(new_n139));
  aoi112aa1n02x5               g044(.a(new_n133), .b(new_n131), .c(new_n126), .d(new_n138), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(\s[11] ));
  oai012aa1n02x5               g046(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .o1(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  aoi012aa1n02x5               g048(.a(new_n99), .b(new_n101), .c(new_n102), .o1(new_n144));
  160nm_ficinv00aa1n08x5       g049(.clk(\a[4] ), .clkout(new_n145));
  nanb02aa1n02x5               g050(.a(\b[3] ), .b(new_n145), .out0(new_n146));
  nanp02aa1n02x5               g051(.a(\b[3] ), .b(\a[4] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n146), .b(new_n147), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(\b[2] ), .b(\a[3] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n109), .b(new_n149), .o1(new_n150));
  norp03aa1n02x5               g055(.a(new_n144), .b(new_n148), .c(new_n150), .o1(new_n151));
  nona23aa1n02x4               g056(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n152));
  norp03aa1n02x5               g057(.a(new_n152), .b(new_n117), .c(new_n118), .o1(new_n153));
  oai012aa1n02x5               g058(.a(new_n153), .b(new_n151), .c(new_n110), .o1(new_n154));
  norp02aa1n02x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[11] ), .b(\a[12] ), .o1(new_n156));
  nona23aa1n02x4               g061(.a(new_n156), .b(new_n130), .c(new_n129), .d(new_n155), .out0(new_n157));
  nano23aa1n02x4               g062(.a(new_n157), .b(new_n137), .c(new_n134), .d(new_n132), .out0(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  nano23aa1n02x4               g064(.a(new_n129), .b(new_n155), .c(new_n156), .d(new_n130), .out0(new_n160));
  aoi012aa1n02x5               g065(.a(new_n155), .b(new_n129), .c(new_n156), .o1(new_n161));
  aobi12aa1n02x5               g066(.a(new_n161), .b(new_n160), .c(new_n133), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n159), .c(new_n154), .d(new_n125), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  aoi012aa1n02x5               g071(.a(new_n165), .b(new_n163), .c(new_n166), .o1(new_n167));
  xnrb03aa1n02x5               g072(.a(new_n167), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n02x4               g073(.a(new_n111), .b(new_n144), .c(new_n148), .d(new_n150), .o1(new_n169));
  oaoi03aa1n02x5               g074(.a(new_n120), .b(new_n121), .c(new_n122), .o1(new_n170));
  oabi12aa1n02x5               g075(.a(new_n124), .b(new_n152), .c(new_n170), .out0(new_n171));
  aoai13aa1n02x5               g076(.a(new_n158), .b(new_n171), .c(new_n169), .d(new_n153), .o1(new_n172));
  norp02aa1n02x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nona23aa1n02x4               g079(.a(new_n174), .b(new_n166), .c(new_n165), .d(new_n173), .out0(new_n175));
  aoi012aa1n02x5               g080(.a(new_n173), .b(new_n165), .c(new_n174), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n175), .c(new_n172), .d(new_n162), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(new_n181), .clkout(new_n182));
  norp02aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n185), .clkout(new_n186));
  aoi112aa1n02x5               g091(.a(new_n186), .b(new_n179), .c(new_n177), .d(new_n182), .o1(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(new_n179), .clkout(new_n188));
  nano23aa1n02x4               g093(.a(new_n165), .b(new_n173), .c(new_n174), .d(new_n166), .out0(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n176), .clkout(new_n190));
  aoai13aa1n02x5               g095(.a(new_n182), .b(new_n190), .c(new_n163), .d(new_n189), .o1(new_n191));
  aoi012aa1n02x5               g096(.a(new_n185), .b(new_n191), .c(new_n188), .o1(new_n192));
  norp02aa1n02x5               g097(.a(new_n192), .b(new_n187), .o1(\s[16] ));
  nona23aa1n02x4               g098(.a(new_n184), .b(new_n180), .c(new_n179), .d(new_n183), .out0(new_n194));
  nona23aa1n02x4               g099(.a(new_n189), .b(new_n138), .c(new_n194), .d(new_n157), .out0(new_n195));
  aoi013aa1n02x4               g100(.a(new_n135), .b(new_n136), .c(new_n97), .d(new_n98), .o1(new_n196));
  oai012aa1n02x5               g101(.a(new_n161), .b(new_n157), .c(new_n196), .o1(new_n197));
  norp02aa1n02x5               g102(.a(new_n194), .b(new_n175), .o1(new_n198));
  aoi012aa1n02x5               g103(.a(new_n183), .b(new_n179), .c(new_n184), .o1(new_n199));
  oai012aa1n02x5               g104(.a(new_n199), .b(new_n194), .c(new_n176), .o1(new_n200));
  aoi012aa1n02x5               g105(.a(new_n200), .b(new_n197), .c(new_n198), .o1(new_n201));
  aoai13aa1n02x5               g106(.a(new_n201), .b(new_n195), .c(new_n154), .d(new_n125), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g108(.clk(\a[18] ), .clkout(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(\a[17] ), .clkout(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(\b[16] ), .clkout(new_n206));
  oaoi03aa1n02x5               g111(.a(new_n205), .b(new_n206), .c(new_n202), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[17] ), .c(new_n204), .out0(\s[18] ));
  nona22aa1n02x4               g113(.a(new_n189), .b(new_n185), .c(new_n181), .out0(new_n209));
  nano22aa1n02x4               g114(.a(new_n209), .b(new_n138), .c(new_n160), .out0(new_n210));
  aoai13aa1n02x5               g115(.a(new_n210), .b(new_n171), .c(new_n169), .d(new_n153), .o1(new_n211));
  xroi22aa1d04x5               g116(.a(new_n205), .b(\b[16] ), .c(new_n204), .d(\b[17] ), .out0(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  norp02aa1n02x5               g118(.a(\b[16] ), .b(\a[17] ), .o1(new_n214));
  norp02aa1n02x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  aoi012aa1n02x5               g121(.a(new_n215), .b(new_n214), .c(new_n216), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n213), .c(new_n211), .d(new_n201), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[19] ), .b(\b[18] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[20] ), .b(\b[19] ), .out0(new_n223));
  aoi112aa1n02x5               g128(.a(new_n221), .b(new_n223), .c(new_n218), .d(new_n222), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n221), .clkout(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n217), .clkout(new_n226));
  aoai13aa1n02x5               g131(.a(new_n222), .b(new_n226), .c(new_n202), .d(new_n212), .o1(new_n227));
  xnrc02aa1n02x5               g132(.a(\b[19] ), .b(\a[20] ), .out0(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n227), .c(new_n225), .o1(new_n229));
  norp02aa1n02x5               g134(.a(new_n229), .b(new_n224), .o1(\s[20] ));
  xnrc02aa1n02x5               g135(.a(\b[18] ), .b(\a[19] ), .out0(new_n231));
  norp02aa1n02x5               g136(.a(new_n228), .b(new_n231), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(new_n212), .b(new_n232), .o1(new_n233));
  oao003aa1n02x5               g138(.a(\a[20] ), .b(\b[19] ), .c(new_n225), .carry(new_n234));
  oai013aa1n02x4               g139(.a(new_n234), .b(new_n231), .c(new_n228), .d(new_n217), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n233), .c(new_n211), .d(new_n201), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  nanb02aa1n02x5               g145(.a(new_n239), .b(new_n240), .out0(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n241), .clkout(new_n242));
  norp02aa1n02x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nanb02aa1n02x5               g149(.a(new_n243), .b(new_n244), .out0(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n245), .clkout(new_n246));
  aoi112aa1n02x5               g151(.a(new_n239), .b(new_n246), .c(new_n237), .d(new_n242), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n239), .clkout(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n233), .clkout(new_n249));
  aoai13aa1n02x5               g154(.a(new_n242), .b(new_n235), .c(new_n202), .d(new_n249), .o1(new_n250));
  aoi012aa1n02x5               g155(.a(new_n245), .b(new_n250), .c(new_n248), .o1(new_n251));
  norp02aa1n02x5               g156(.a(new_n251), .b(new_n247), .o1(\s[22] ));
  nano23aa1n02x4               g157(.a(new_n239), .b(new_n243), .c(new_n244), .d(new_n240), .out0(new_n253));
  nanp03aa1n02x5               g158(.a(new_n212), .b(new_n232), .c(new_n253), .o1(new_n254));
  aoi012aa1n02x5               g159(.a(new_n243), .b(new_n239), .c(new_n244), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n255), .clkout(new_n256));
  aoi012aa1n02x5               g161(.a(new_n256), .b(new_n235), .c(new_n253), .o1(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n254), .c(new_n211), .d(new_n201), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g164(.a(\b[22] ), .b(\a[23] ), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n261), .b(new_n260), .out0(new_n262));
  xorc02aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .out0(new_n263));
  aoi112aa1n02x5               g168(.a(new_n260), .b(new_n263), .c(new_n258), .d(new_n262), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n260), .clkout(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n254), .clkout(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n257), .clkout(new_n267));
  aoai13aa1n02x5               g172(.a(new_n262), .b(new_n267), .c(new_n202), .d(new_n266), .o1(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[23] ), .b(\a[24] ), .out0(new_n269));
  aoi012aa1n02x5               g174(.a(new_n269), .b(new_n268), .c(new_n265), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n270), .b(new_n264), .o1(\s[24] ));
  nano22aa1n02x4               g176(.a(new_n269), .b(new_n265), .c(new_n261), .out0(new_n272));
  nano22aa1n02x4               g177(.a(new_n233), .b(new_n253), .c(new_n272), .out0(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  nanb03aa1n02x5               g179(.a(new_n217), .b(new_n223), .c(new_n222), .out0(new_n275));
  nanp03aa1n02x5               g180(.a(new_n253), .b(new_n262), .c(new_n263), .o1(new_n276));
  oaoi03aa1n02x5               g181(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .o1(new_n277));
  aoi013aa1n02x4               g182(.a(new_n277), .b(new_n256), .c(new_n262), .d(new_n263), .o1(new_n278));
  aoai13aa1n02x5               g183(.a(new_n278), .b(new_n276), .c(new_n275), .d(new_n234), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n279), .clkout(new_n280));
  aoai13aa1n02x5               g185(.a(new_n280), .b(new_n274), .c(new_n211), .d(new_n201), .o1(new_n281));
  xorb03aa1n02x5               g186(.a(new_n281), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[25] ), .b(\b[24] ), .out0(new_n284));
  xorc02aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  aoi112aa1n02x5               g190(.a(new_n283), .b(new_n285), .c(new_n281), .d(new_n284), .o1(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n283), .clkout(new_n287));
  aoai13aa1n02x5               g192(.a(new_n284), .b(new_n279), .c(new_n202), .d(new_n273), .o1(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(new_n285), .clkout(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n288), .c(new_n287), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n290), .b(new_n286), .o1(\s[26] ));
  oabi12aa1n02x5               g196(.a(new_n200), .b(new_n162), .c(new_n209), .out0(new_n292));
  and002aa1n02x5               g197(.a(new_n285), .b(new_n284), .o(new_n293));
  nano22aa1n02x4               g198(.a(new_n254), .b(new_n293), .c(new_n272), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n126), .d(new_n210), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n296));
  aobi12aa1n02x5               g201(.a(new_n296), .b(new_n279), .c(new_n293), .out0(new_n297));
  xorc02aa1n02x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  xnbna2aa1n03x5               g203(.a(new_n298), .b(new_n295), .c(new_n297), .out0(\s[27] ));
  norp02aa1n02x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  160nm_ficinv00aa1n08x5       g205(.clk(new_n300), .clkout(new_n301));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n298), .clkout(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n295), .c(new_n297), .o1(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[27] ), .b(\a[28] ), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n303), .b(new_n301), .c(new_n304), .out0(new_n305));
  nona23aa1n02x4               g210(.a(new_n244), .b(new_n240), .c(new_n239), .d(new_n243), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n306), .b(new_n263), .c(new_n262), .out0(new_n307));
  nanp02aa1n02x5               g212(.a(new_n235), .b(new_n307), .o1(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n293), .clkout(new_n309));
  aoai13aa1n02x5               g214(.a(new_n296), .b(new_n309), .c(new_n308), .d(new_n278), .o1(new_n310));
  aoai13aa1n02x5               g215(.a(new_n298), .b(new_n310), .c(new_n202), .d(new_n294), .o1(new_n311));
  aoi012aa1n02x5               g216(.a(new_n304), .b(new_n311), .c(new_n301), .o1(new_n312));
  norp02aa1n02x5               g217(.a(new_n312), .b(new_n305), .o1(\s[28] ));
  norb02aa1n02x5               g218(.a(new_n298), .b(new_n304), .out0(new_n314));
  aoai13aa1n02x5               g219(.a(new_n314), .b(new_n310), .c(new_n202), .d(new_n294), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[28] ), .b(\b[27] ), .c(new_n301), .carry(new_n316));
  xnrc02aa1n02x5               g221(.a(\b[28] ), .b(\a[29] ), .out0(new_n317));
  aoi012aa1n02x5               g222(.a(new_n317), .b(new_n315), .c(new_n316), .o1(new_n318));
  160nm_ficinv00aa1n08x5       g223(.clk(new_n314), .clkout(new_n319));
  aoi012aa1n02x5               g224(.a(new_n319), .b(new_n295), .c(new_n297), .o1(new_n320));
  nano22aa1n02x4               g225(.a(new_n320), .b(new_n316), .c(new_n317), .out0(new_n321));
  norp02aa1n02x5               g226(.a(new_n318), .b(new_n321), .o1(\s[29] ));
  xorb03aa1n02x5               g227(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g228(.a(new_n298), .b(new_n317), .c(new_n304), .out0(new_n324));
  aoai13aa1n02x5               g229(.a(new_n324), .b(new_n310), .c(new_n202), .d(new_n294), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[29] ), .b(\b[28] ), .c(new_n316), .carry(new_n326));
  xnrc02aa1n02x5               g231(.a(\b[29] ), .b(\a[30] ), .out0(new_n327));
  aoi012aa1n02x5               g232(.a(new_n327), .b(new_n325), .c(new_n326), .o1(new_n328));
  160nm_ficinv00aa1n08x5       g233(.clk(new_n324), .clkout(new_n329));
  aoi012aa1n02x5               g234(.a(new_n329), .b(new_n295), .c(new_n297), .o1(new_n330));
  nano22aa1n02x4               g235(.a(new_n330), .b(new_n326), .c(new_n327), .out0(new_n331));
  norp02aa1n02x5               g236(.a(new_n328), .b(new_n331), .o1(\s[30] ));
  xnrc02aa1n02x5               g237(.a(\b[30] ), .b(\a[31] ), .out0(new_n333));
  nona32aa1n02x4               g238(.a(new_n298), .b(new_n327), .c(new_n317), .d(new_n304), .out0(new_n334));
  aoi012aa1n02x5               g239(.a(new_n334), .b(new_n295), .c(new_n297), .o1(new_n335));
  oao003aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .c(new_n326), .carry(new_n336));
  nano22aa1n02x4               g241(.a(new_n335), .b(new_n333), .c(new_n336), .out0(new_n337));
  160nm_ficinv00aa1n08x5       g242(.clk(new_n334), .clkout(new_n338));
  aoai13aa1n02x5               g243(.a(new_n338), .b(new_n310), .c(new_n202), .d(new_n294), .o1(new_n339));
  aoi012aa1n02x5               g244(.a(new_n333), .b(new_n339), .c(new_n336), .o1(new_n340));
  norp02aa1n02x5               g245(.a(new_n340), .b(new_n337), .o1(\s[31] ));
  xnbna2aa1n03x5               g246(.a(new_n144), .b(new_n149), .c(new_n109), .out0(\s[3] ));
  oaoi03aa1n02x5               g247(.a(\a[3] ), .b(\b[2] ), .c(new_n144), .o1(new_n343));
  xorb03aa1n02x5               g248(.a(new_n343), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g249(.a(new_n169), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g250(.a(\b[4] ), .b(\a[5] ), .o1(new_n346));
  aoi012aa1n02x5               g251(.a(new_n122), .b(new_n169), .c(new_n346), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[5] ), .c(new_n120), .out0(\s[6] ));
  oaoi03aa1n02x5               g253(.a(\a[6] ), .b(\b[5] ), .c(new_n347), .o1(new_n349));
  xorb03aa1n02x5               g254(.a(new_n349), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g255(.a(new_n114), .b(new_n349), .c(new_n115), .o1(new_n351));
  xnrb03aa1n02x5               g256(.a(new_n351), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g257(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


