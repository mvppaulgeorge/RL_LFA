// Benchmark "adder" written by ABC on Thu Jul 11 11:56:03 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n323, new_n325, new_n326;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  nanp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[2] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\b[1] ), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  oaoi03aa1n02x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n02x4               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  oai012aa1n02x5               g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai012aa1n02x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n02x4               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  norp03aa1n02x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\a[6] ), .clkout(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\b[5] ), .clkout(new_n120));
  norp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  oa0012aa1n02x5               g027(.a(new_n112), .b(new_n113), .c(new_n111), .o(new_n123));
  oabi12aa1n02x5               g028(.a(new_n123), .b(new_n115), .c(new_n122), .out0(new_n124));
  aoi112aa1n02x5               g029(.a(new_n124), .b(new_n99), .c(new_n110), .d(new_n118), .o1(new_n125));
  nano22aa1n02x4               g030(.a(new_n125), .b(new_n97), .c(new_n98), .out0(new_n126));
  oao003aa1n02x5               g031(.a(new_n100), .b(new_n101), .c(new_n102), .carry(new_n127));
  nano23aa1n02x4               g032(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n128));
  aobi12aa1n02x5               g033(.a(new_n109), .b(new_n128), .c(new_n127), .out0(new_n129));
  nano23aa1n02x4               g034(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n130));
  nona22aa1n02x4               g035(.a(new_n130), .b(new_n116), .c(new_n117), .out0(new_n131));
  oao003aa1n02x5               g036(.a(new_n119), .b(new_n120), .c(new_n121), .carry(new_n132));
  aoi012aa1n02x5               g037(.a(new_n123), .b(new_n130), .c(new_n132), .o1(new_n133));
  oai012aa1n02x5               g038(.a(new_n133), .b(new_n129), .c(new_n131), .o1(new_n134));
  oaoi13aa1n02x5               g039(.a(new_n97), .b(new_n98), .c(new_n134), .d(new_n99), .o1(new_n135));
  norp02aa1n02x5               g040(.a(new_n135), .b(new_n126), .o1(\s[10] ));
  norp02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n137), .clkout(new_n138));
  nanp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  160nm_ficinv00aa1n08x5       g044(.clk(\a[10] ), .clkout(new_n140));
  160nm_ficinv00aa1n08x5       g045(.clk(\b[9] ), .clkout(new_n141));
  oao003aa1n02x5               g046(.a(new_n140), .b(new_n141), .c(new_n99), .carry(new_n142));
  norp02aa1n02x5               g047(.a(new_n126), .b(new_n142), .o1(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n138), .c(new_n139), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g049(.clk(\a[12] ), .clkout(new_n145));
  oaoi13aa1n02x5               g050(.a(new_n137), .b(new_n139), .c(new_n126), .d(new_n142), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[11] ), .c(new_n145), .out0(\s[12] ));
  nanp02aa1n02x5               g052(.a(new_n110), .b(new_n118), .o1(new_n148));
  norp02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nano23aa1n02x4               g055(.a(new_n137), .b(new_n149), .c(new_n150), .d(new_n139), .out0(new_n151));
  norb02aa1n02x5               g056(.a(new_n98), .b(new_n99), .out0(new_n152));
  nanp03aa1n02x5               g057(.a(new_n151), .b(new_n97), .c(new_n152), .o1(new_n153));
  oai012aa1n02x5               g058(.a(new_n150), .b(new_n149), .c(new_n137), .o1(new_n154));
  aobi12aa1n02x5               g059(.a(new_n154), .b(new_n151), .c(new_n142), .out0(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n153), .c(new_n148), .d(new_n133), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g062(.clk(\a[14] ), .clkout(new_n158));
  norp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  xorc02aa1n02x5               g064(.a(\a[13] ), .b(\b[12] ), .out0(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n156), .c(new_n160), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(new_n158), .out0(\s[14] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  xorc02aa1n02x5               g070(.a(\a[14] ), .b(\b[13] ), .out0(new_n166));
  and002aa1n02x5               g071(.a(new_n166), .b(new_n160), .o(new_n167));
  160nm_ficinv00aa1n08x5       g072(.clk(\b[13] ), .clkout(new_n168));
  oaoi03aa1n02x5               g073(.a(new_n158), .b(new_n168), .c(new_n159), .o1(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(new_n169), .clkout(new_n170));
  aoai13aa1n02x5               g075(.a(new_n165), .b(new_n170), .c(new_n156), .d(new_n167), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n165), .b(new_n170), .c(new_n156), .d(new_n167), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[15] ));
  norp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  nona22aa1n02x4               g081(.a(new_n171), .b(new_n176), .c(new_n163), .out0(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(new_n176), .clkout(new_n178));
  oaoi13aa1n02x5               g083(.a(new_n178), .b(new_n171), .c(\a[15] ), .d(\b[14] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n177), .b(new_n179), .out0(\s[16] ));
  nano23aa1n02x4               g085(.a(new_n163), .b(new_n174), .c(new_n175), .d(new_n164), .out0(new_n181));
  nanp03aa1n02x5               g086(.a(new_n181), .b(new_n160), .c(new_n166), .o1(new_n182));
  norp02aa1n02x5               g087(.a(new_n182), .b(new_n153), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n184));
  oaoi03aa1n02x5               g089(.a(new_n140), .b(new_n141), .c(new_n99), .o1(new_n185));
  nona23aa1n02x4               g090(.a(new_n150), .b(new_n139), .c(new_n137), .d(new_n149), .out0(new_n186));
  oai012aa1n02x5               g091(.a(new_n154), .b(new_n186), .c(new_n185), .o1(new_n187));
  oai012aa1n02x5               g092(.a(new_n175), .b(new_n174), .c(new_n163), .o1(new_n188));
  oaib12aa1n02x5               g093(.a(new_n188), .b(new_n169), .c(new_n181), .out0(new_n189));
  aoib12aa1n02x5               g094(.a(new_n189), .b(new_n187), .c(new_n182), .out0(new_n190));
  nanp02aa1n02x5               g095(.a(new_n184), .b(new_n190), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g097(.clk(\a[18] ), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(\a[17] ), .clkout(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(\b[16] ), .clkout(new_n195));
  oaoi03aa1n02x5               g100(.a(new_n194), .b(new_n195), .c(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  xroi22aa1d04x5               g102(.a(new_n194), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n198));
  norp02aa1n02x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  aoi112aa1n02x5               g104(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n200));
  norp02aa1n02x5               g105(.a(new_n200), .b(new_n199), .o1(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n201), .clkout(new_n202));
  norp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  aoai13aa1n02x5               g110(.a(new_n205), .b(new_n202), .c(new_n191), .d(new_n198), .o1(new_n206));
  aoi112aa1n02x5               g111(.a(new_n205), .b(new_n202), .c(new_n191), .d(new_n198), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  nona22aa1n02x4               g117(.a(new_n206), .b(new_n212), .c(new_n203), .out0(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n203), .clkout(new_n214));
  aobi12aa1n02x5               g119(.a(new_n212), .b(new_n206), .c(new_n214), .out0(new_n215));
  norb02aa1n02x5               g120(.a(new_n213), .b(new_n215), .out0(\s[20] ));
  nona23aa1n02x4               g121(.a(new_n211), .b(new_n204), .c(new_n203), .d(new_n210), .out0(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  nanp02aa1n02x5               g123(.a(new_n198), .b(new_n218), .o1(new_n219));
  oai012aa1n02x5               g124(.a(new_n211), .b(new_n210), .c(new_n203), .o1(new_n220));
  oai012aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n201), .o1(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(new_n221), .clkout(new_n222));
  aoai13aa1n02x5               g127(.a(new_n222), .b(new_n219), .c(new_n184), .d(new_n190), .o1(new_n223));
  xorb03aa1n02x5               g128(.a(new_n223), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  xorc02aa1n02x5               g130(.a(\a[21] ), .b(\b[20] ), .out0(new_n226));
  xorc02aa1n02x5               g131(.a(\a[22] ), .b(\b[21] ), .out0(new_n227));
  aoi112aa1n02x5               g132(.a(new_n225), .b(new_n227), .c(new_n223), .d(new_n226), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n227), .b(new_n225), .c(new_n223), .d(new_n226), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n229), .b(new_n228), .out0(\s[22] ));
  nanp02aa1n02x5               g135(.a(new_n227), .b(new_n226), .o1(new_n231));
  nanb03aa1n02x5               g136(.a(new_n231), .b(new_n198), .c(new_n218), .out0(new_n232));
  oai112aa1n02x5               g137(.a(new_n205), .b(new_n212), .c(new_n200), .d(new_n199), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\a[22] ), .clkout(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(\b[21] ), .clkout(new_n235));
  oao003aa1n02x5               g140(.a(new_n234), .b(new_n235), .c(new_n225), .carry(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  aoai13aa1n02x5               g142(.a(new_n237), .b(new_n231), .c(new_n233), .d(new_n220), .o1(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n232), .c(new_n184), .d(new_n190), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  xorc02aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n242), .b(new_n244), .c(new_n240), .d(new_n243), .o1(new_n245));
  aoai13aa1n02x5               g150(.a(new_n244), .b(new_n242), .c(new_n240), .d(new_n243), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(\s[24] ));
  and002aa1n02x5               g152(.a(new_n244), .b(new_n243), .o(new_n248));
  nona23aa1n02x4               g153(.a(new_n248), .b(new_n198), .c(new_n231), .d(new_n217), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(\a[24] ), .clkout(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(\b[23] ), .clkout(new_n251));
  oao003aa1n02x5               g156(.a(new_n250), .b(new_n251), .c(new_n242), .carry(new_n252));
  aoi012aa1n02x5               g157(.a(new_n252), .b(new_n238), .c(new_n248), .o1(new_n253));
  aoai13aa1n02x5               g158(.a(new_n253), .b(new_n249), .c(new_n184), .d(new_n190), .o1(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xorc02aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .out0(new_n258));
  aoi112aa1n02x5               g163(.a(new_n256), .b(new_n258), .c(new_n254), .d(new_n257), .o1(new_n259));
  aoai13aa1n02x5               g164(.a(new_n258), .b(new_n256), .c(new_n254), .d(new_n257), .o1(new_n260));
  norb02aa1n02x5               g165(.a(new_n260), .b(new_n259), .out0(\s[26] ));
  aobi12aa1n02x5               g166(.a(new_n188), .b(new_n170), .c(new_n181), .out0(new_n262));
  oai012aa1n02x5               g167(.a(new_n262), .b(new_n155), .c(new_n182), .o1(new_n263));
  and002aa1n02x5               g168(.a(new_n258), .b(new_n257), .o(new_n264));
  nano22aa1n02x4               g169(.a(new_n232), .b(new_n248), .c(new_n264), .out0(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n263), .c(new_n134), .d(new_n183), .o1(new_n266));
  aoai13aa1n02x5               g171(.a(new_n264), .b(new_n252), .c(new_n238), .d(new_n248), .o1(new_n267));
  oai022aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n268));
  aob012aa1n02x5               g173(.a(new_n268), .b(\b[25] ), .c(\a[26] ), .out0(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  aoi013aa1n02x4               g176(.a(new_n271), .b(new_n266), .c(new_n267), .d(new_n269), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n265), .clkout(new_n273));
  aoi012aa1n02x5               g178(.a(new_n273), .b(new_n184), .c(new_n190), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n231), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n248), .b(new_n236), .c(new_n221), .d(new_n275), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n252), .clkout(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n264), .clkout(new_n278));
  aoai13aa1n02x5               g183(.a(new_n269), .b(new_n278), .c(new_n276), .d(new_n277), .o1(new_n279));
  norp03aa1n02x5               g184(.a(new_n279), .b(new_n274), .c(new_n270), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n272), .b(new_n280), .o1(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n282), .clkout(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n272), .b(new_n283), .c(new_n284), .out0(new_n285));
  oai012aa1n02x5               g190(.a(new_n270), .b(new_n279), .c(new_n274), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n284), .b(new_n286), .c(new_n283), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g193(.a(new_n270), .b(new_n284), .out0(new_n289));
  oai012aa1n02x5               g194(.a(new_n289), .b(new_n279), .c(new_n274), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n289), .clkout(new_n294));
  aoi013aa1n02x4               g199(.a(new_n294), .b(new_n266), .c(new_n267), .d(new_n269), .o1(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n291), .c(new_n292), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n293), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n270), .b(new_n292), .c(new_n284), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n299), .b(new_n279), .c(new_n274), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n299), .clkout(new_n304));
  aoi013aa1n02x4               g209(.a(new_n304), .b(new_n266), .c(new_n267), .d(new_n269), .o1(new_n305));
  nano22aa1n02x4               g210(.a(new_n305), .b(new_n301), .c(new_n302), .out0(new_n306));
  norp02aa1n02x5               g211(.a(new_n303), .b(new_n306), .o1(\s[30] ));
  norb02aa1n02x5               g212(.a(new_n299), .b(new_n302), .out0(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n308), .clkout(new_n309));
  aoi013aa1n02x4               g214(.a(new_n309), .b(new_n266), .c(new_n267), .d(new_n269), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n310), .b(new_n311), .c(new_n312), .out0(new_n313));
  oai012aa1n02x5               g218(.a(new_n308), .b(new_n279), .c(new_n274), .o1(new_n314));
  aoi012aa1n02x5               g219(.a(new_n312), .b(new_n314), .c(new_n311), .o1(new_n315));
  norp02aa1n02x5               g220(.a(new_n315), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g225(.a(\a[5] ), .b(\b[4] ), .c(new_n129), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g227(.a(new_n119), .b(new_n120), .c(new_n321), .carry(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  160nm_ficinv00aa1n08x5       g229(.clk(\a[8] ), .clkout(new_n325));
  aoi012aa1n02x5               g230(.a(new_n113), .b(new_n323), .c(new_n114), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[7] ), .c(new_n325), .out0(\s[8] ));
  xnbna2aa1n03x5               g232(.a(new_n152), .b(new_n148), .c(new_n133), .out0(\s[9] ));
endmodule


