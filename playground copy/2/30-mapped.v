// Benchmark "adder" written by ABC on Wed Jul 10 17:25:39 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n321, new_n324, new_n326, new_n327,
    new_n329;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[2] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\b[1] ), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  oaoi03aa1n02x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n02x4               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  aoi012aa1n02x5               g013(.a(new_n104), .b(new_n106), .c(new_n105), .o1(new_n109));
  oai012aa1n02x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  norb02aa1n02x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  norp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norb02aa1n02x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  nano23aa1n02x4               g023(.a(new_n118), .b(new_n117), .c(new_n116), .d(new_n113), .out0(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n110), .o1(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(new_n114), .clkout(new_n121));
  aoi112aa1n02x5               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(new_n122), .clkout(new_n123));
  nanb02aa1n02x5               g028(.a(new_n114), .b(new_n115), .out0(new_n124));
  oai022aa1n02x5               g029(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n125));
  nano23aa1n02x4               g030(.a(new_n117), .b(new_n124), .c(new_n125), .d(new_n112), .out0(new_n126));
  nano22aa1n02x4               g031(.a(new_n126), .b(new_n121), .c(new_n123), .out0(new_n127));
  xnrc02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n99), .b(new_n128), .c(new_n120), .d(new_n127), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  oao003aa1n02x5               g035(.a(new_n100), .b(new_n101), .c(new_n102), .carry(new_n131));
  nano23aa1n02x4               g036(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n132));
  aobi12aa1n02x5               g037(.a(new_n109), .b(new_n132), .c(new_n131), .out0(new_n133));
  xorc02aa1n02x5               g038(.a(\a[7] ), .b(\b[6] ), .out0(new_n134));
  nona23aa1n02x4               g039(.a(new_n134), .b(new_n113), .c(new_n118), .d(new_n124), .out0(new_n135));
  oai012aa1n02x5               g040(.a(new_n127), .b(new_n133), .c(new_n135), .o1(new_n136));
  oaoi03aa1n02x5               g041(.a(new_n97), .b(new_n98), .c(new_n136), .o1(new_n137));
  norp02aa1n02x5               g042(.a(\b[9] ), .b(\a[10] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[9] ), .b(\a[10] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  norp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  160nm_ficinv00aa1n08x5       g048(.clk(new_n143), .clkout(new_n144));
  aoai13aa1n02x5               g049(.a(new_n139), .b(new_n138), .c(new_n97), .d(new_n98), .o1(new_n145));
  oaoi13aa1n02x5               g050(.a(new_n144), .b(new_n145), .c(new_n137), .d(new_n140), .o1(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n140), .clkout(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n145), .clkout(new_n148));
  aoi112aa1n02x5               g053(.a(new_n148), .b(new_n143), .c(new_n129), .d(new_n147), .o1(new_n149));
  norp02aa1n02x5               g054(.a(new_n146), .b(new_n149), .o1(\s[11] ));
  aoai13aa1n02x5               g055(.a(new_n143), .b(new_n148), .c(new_n129), .d(new_n147), .o1(new_n151));
  norp02aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  oai112aa1n02x5               g059(.a(new_n151), .b(new_n154), .c(\b[10] ), .d(\a[11] ), .o1(new_n155));
  oaoi13aa1n02x5               g060(.a(new_n154), .b(new_n151), .c(\a[11] ), .d(\b[10] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(\s[12] ));
  nano23aa1n02x4               g062(.a(new_n141), .b(new_n152), .c(new_n153), .d(new_n142), .out0(new_n158));
  nona22aa1n02x4               g063(.a(new_n158), .b(new_n128), .c(new_n140), .out0(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n152), .clkout(new_n160));
  nona23aa1n02x4               g065(.a(new_n153), .b(new_n142), .c(new_n141), .d(new_n152), .out0(new_n161));
  nanp02aa1n02x5               g066(.a(new_n141), .b(new_n153), .o1(new_n162));
  oai112aa1n02x5               g067(.a(new_n162), .b(new_n160), .c(new_n161), .d(new_n145), .o1(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n159), .c(new_n120), .d(new_n127), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  aoi012aa1n02x5               g073(.a(new_n167), .b(new_n165), .c(new_n168), .o1(new_n169));
  xnrb03aa1n02x5               g074(.a(new_n169), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n173), .clkout(new_n174));
  norp02aa1n02x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[13] ), .b(\a[14] ), .o1(new_n176));
  nano23aa1n02x4               g081(.a(new_n167), .b(new_n175), .c(new_n176), .d(new_n168), .out0(new_n177));
  oai012aa1n02x5               g082(.a(new_n176), .b(new_n175), .c(new_n167), .o1(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n178), .clkout(new_n179));
  aoai13aa1n02x5               g084(.a(new_n174), .b(new_n179), .c(new_n165), .d(new_n177), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n174), .b(new_n179), .c(new_n165), .d(new_n177), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n180), .b(new_n181), .out0(\s[15] ));
  norp02aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  nona22aa1n02x4               g090(.a(new_n180), .b(new_n185), .c(new_n171), .out0(new_n186));
  nanb02aa1n02x5               g091(.a(new_n183), .b(new_n184), .out0(new_n187));
  oaoi13aa1n02x5               g092(.a(new_n187), .b(new_n180), .c(\a[15] ), .d(\b[14] ), .o1(new_n188));
  norb02aa1n02x5               g093(.a(new_n186), .b(new_n188), .out0(\s[16] ));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n112), .clkout(new_n190));
  norp02aa1n02x5               g095(.a(\b[4] ), .b(\a[5] ), .o1(new_n191));
  norp02aa1n02x5               g096(.a(new_n191), .b(new_n111), .o1(new_n192));
  nona23aa1n02x4               g097(.a(new_n134), .b(new_n116), .c(new_n192), .d(new_n190), .out0(new_n193));
  nona22aa1n02x4               g098(.a(new_n193), .b(new_n122), .c(new_n114), .out0(new_n194));
  nano32aa1n02x4               g099(.a(new_n159), .b(new_n185), .c(new_n174), .d(new_n177), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n194), .c(new_n110), .d(new_n119), .o1(new_n196));
  nona23aa1n02x4               g101(.a(new_n184), .b(new_n172), .c(new_n171), .d(new_n183), .out0(new_n197));
  norb02aa1n02x5               g102(.a(new_n177), .b(new_n197), .out0(new_n198));
  aoi112aa1n02x5               g103(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n199));
  oai022aa1n02x5               g104(.a(new_n197), .b(new_n178), .c(\b[15] ), .d(\a[16] ), .o1(new_n200));
  aoi112aa1n02x5               g105(.a(new_n200), .b(new_n199), .c(new_n163), .d(new_n198), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(new_n201), .b(new_n196), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g108(.clk(\a[18] ), .clkout(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(\a[17] ), .clkout(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(\b[16] ), .clkout(new_n206));
  oaoi03aa1n02x5               g111(.a(new_n205), .b(new_n206), .c(new_n202), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[17] ), .c(new_n204), .out0(\s[18] ));
  xroi22aa1d04x5               g113(.a(new_n205), .b(\b[16] ), .c(new_n204), .d(\b[17] ), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n206), .b(new_n205), .o1(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n210), .o1(new_n211));
  norp02aa1n02x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n211), .c(new_n202), .d(new_n209), .o1(new_n215));
  aoi112aa1n02x5               g120(.a(new_n214), .b(new_n211), .c(new_n202), .d(new_n209), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n215), .b(new_n216), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  nona22aa1n02x4               g126(.a(new_n215), .b(new_n221), .c(new_n212), .out0(new_n222));
  orn002aa1n02x5               g127(.a(\a[19] ), .b(\b[18] ), .o(new_n223));
  aobi12aa1n02x5               g128(.a(new_n221), .b(new_n215), .c(new_n223), .out0(new_n224));
  norb02aa1n02x5               g129(.a(new_n222), .b(new_n224), .out0(\s[20] ));
  nano23aa1n02x4               g130(.a(new_n212), .b(new_n219), .c(new_n220), .d(new_n213), .out0(new_n226));
  nanp02aa1n02x5               g131(.a(new_n209), .b(new_n226), .o1(new_n227));
  oai022aa1n02x5               g132(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n228));
  oaib12aa1n02x5               g133(.a(new_n228), .b(new_n204), .c(\b[17] ), .out0(new_n229));
  nona23aa1n02x4               g134(.a(new_n220), .b(new_n213), .c(new_n212), .d(new_n219), .out0(new_n230));
  oaoi03aa1n02x5               g135(.a(\a[20] ), .b(\b[19] ), .c(new_n223), .o1(new_n231));
  oabi12aa1n02x5               g136(.a(new_n231), .b(new_n230), .c(new_n229), .out0(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  aoai13aa1n02x5               g138(.a(new_n233), .b(new_n227), .c(new_n201), .d(new_n196), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  xorc02aa1n02x5               g141(.a(\a[21] ), .b(\b[20] ), .out0(new_n237));
  xorc02aa1n02x5               g142(.a(\a[22] ), .b(\b[21] ), .out0(new_n238));
  aoi112aa1n02x5               g143(.a(new_n236), .b(new_n238), .c(new_n234), .d(new_n237), .o1(new_n239));
  aoai13aa1n02x5               g144(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n240));
  norb02aa1n02x5               g145(.a(new_n240), .b(new_n239), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g146(.clk(\a[21] ), .clkout(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(\a[22] ), .clkout(new_n243));
  xroi22aa1d04x5               g148(.a(new_n242), .b(\b[20] ), .c(new_n243), .d(\b[21] ), .out0(new_n244));
  nanp03aa1n02x5               g149(.a(new_n244), .b(new_n209), .c(new_n226), .o1(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(\b[21] ), .clkout(new_n246));
  oaoi03aa1n02x5               g151(.a(new_n243), .b(new_n246), .c(new_n236), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n247), .clkout(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n232), .c(new_n244), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n245), .c(new_n201), .d(new_n196), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[23] ), .b(\b[22] ), .out0(new_n253));
  xorc02aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n252), .b(new_n254), .c(new_n250), .d(new_n253), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n254), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(\s[24] ));
  and002aa1n02x5               g162(.a(new_n254), .b(new_n253), .o(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n258), .clkout(new_n259));
  nano32aa1n02x4               g164(.a(new_n259), .b(new_n244), .c(new_n209), .d(new_n226), .out0(new_n260));
  aoai13aa1n02x5               g165(.a(new_n244), .b(new_n231), .c(new_n226), .d(new_n211), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n262));
  oab012aa1n02x4               g167(.a(new_n262), .b(\a[24] ), .c(\b[23] ), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n259), .c(new_n261), .d(new_n247), .o1(new_n264));
  xorc02aa1n02x5               g169(.a(\a[25] ), .b(\b[24] ), .out0(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n264), .c(new_n202), .d(new_n260), .o1(new_n266));
  aoi112aa1n02x5               g171(.a(new_n265), .b(new_n264), .c(new_n202), .d(new_n260), .o1(new_n267));
  norb02aa1n02x5               g172(.a(new_n266), .b(new_n267), .out0(\s[25] ));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  nona22aa1n02x4               g175(.a(new_n266), .b(new_n270), .c(new_n269), .out0(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n269), .clkout(new_n272));
  aobi12aa1n02x5               g177(.a(new_n270), .b(new_n266), .c(new_n272), .out0(new_n273));
  norb02aa1n02x5               g178(.a(new_n271), .b(new_n273), .out0(\s[26] ));
  and002aa1n02x5               g179(.a(new_n270), .b(new_n265), .o(new_n275));
  nano22aa1n02x4               g180(.a(new_n245), .b(new_n258), .c(new_n275), .out0(new_n276));
  nanp02aa1n02x5               g181(.a(new_n202), .b(new_n276), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n272), .carry(new_n278));
  aobi12aa1n02x5               g183(.a(new_n278), .b(new_n264), .c(new_n275), .out0(new_n279));
  xorc02aa1n02x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n279), .out0(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n282), .clkout(new_n283));
  aobi12aa1n02x5               g188(.a(new_n280), .b(new_n277), .c(new_n279), .out0(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n284), .b(new_n283), .c(new_n285), .out0(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n276), .clkout(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n201), .c(new_n196), .o1(new_n288));
  aoai13aa1n02x5               g193(.a(new_n258), .b(new_n248), .c(new_n232), .d(new_n244), .o1(new_n289));
  160nm_ficinv00aa1n08x5       g194(.clk(new_n275), .clkout(new_n290));
  aoai13aa1n02x5               g195(.a(new_n278), .b(new_n290), .c(new_n289), .d(new_n263), .o1(new_n291));
  oai012aa1n02x5               g196(.a(new_n280), .b(new_n291), .c(new_n288), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n285), .b(new_n292), .c(new_n283), .o1(new_n293));
  norp02aa1n02x5               g198(.a(new_n293), .b(new_n286), .o1(\s[28] ));
  norb02aa1n02x5               g199(.a(new_n280), .b(new_n285), .out0(new_n295));
  oai012aa1n02x5               g200(.a(new_n295), .b(new_n291), .c(new_n288), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[28] ), .b(\a[29] ), .out0(new_n298));
  aoi012aa1n02x5               g203(.a(new_n298), .b(new_n296), .c(new_n297), .o1(new_n299));
  aobi12aa1n02x5               g204(.a(new_n295), .b(new_n277), .c(new_n279), .out0(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n297), .c(new_n298), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n299), .b(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g207(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g208(.a(new_n280), .b(new_n298), .c(new_n285), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n304), .b(new_n291), .c(new_n288), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[29] ), .b(\a[30] ), .out0(new_n307));
  aoi012aa1n02x5               g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n02x5               g213(.a(new_n304), .b(new_n277), .c(new_n279), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  norp02aa1n02x5               g215(.a(new_n308), .b(new_n310), .o1(\s[30] ));
  norb02aa1n02x5               g216(.a(new_n304), .b(new_n307), .out0(new_n312));
  aobi12aa1n02x5               g217(.a(new_n312), .b(new_n277), .c(new_n279), .out0(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  nano22aa1n02x4               g220(.a(new_n313), .b(new_n314), .c(new_n315), .out0(new_n316));
  oai012aa1n02x5               g221(.a(new_n312), .b(new_n291), .c(new_n288), .o1(new_n317));
  aoi012aa1n02x5               g222(.a(new_n315), .b(new_n317), .c(new_n314), .o1(new_n318));
  norp02aa1n02x5               g223(.a(new_n318), .b(new_n316), .o1(\s[31] ));
  xnrb03aa1n02x5               g224(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g225(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g227(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g228(.a(\a[5] ), .b(\b[4] ), .c(new_n133), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g230(.a(new_n134), .b(new_n111), .c(new_n324), .d(new_n112), .o1(new_n326));
  aoi112aa1n02x5               g231(.a(new_n134), .b(new_n111), .c(new_n324), .d(new_n112), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n326), .b(new_n327), .out0(\s[7] ));
  orn002aa1n02x5               g233(.a(\a[7] ), .b(\b[6] ), .o(new_n329));
  xnbna2aa1n03x5               g234(.a(new_n116), .b(new_n326), .c(new_n329), .out0(\s[8] ));
  xobna2aa1n03x5               g235(.a(new_n128), .b(new_n120), .c(new_n127), .out0(\s[9] ));
endmodule


