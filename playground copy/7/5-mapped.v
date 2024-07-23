// Benchmark "adder" written by ABC on Thu Jul 11 11:22:03 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n331,
    new_n334, new_n336, new_n338;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1n02x4               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  norp02aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nanb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .out0(new_n105));
  norp03aa1n02x5               g010(.a(new_n101), .b(new_n104), .c(new_n105), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  aoi012aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nona23aa1n02x4               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  aoi012aa1n02x5               g020(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n116));
  oai012aa1n02x5               g021(.a(new_n116), .b(new_n115), .c(new_n110), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(new_n97), .clkout(new_n118));
  nanp02aa1n02x5               g023(.a(new_n99), .b(new_n98), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n103), .b(new_n120), .c(new_n102), .o1(new_n121));
  oai112aa1n02x5               g026(.a(new_n118), .b(new_n119), .c(new_n101), .d(new_n121), .o1(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n117), .c(new_n106), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .c(new_n123), .o1(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanb02aa1n02x5               g030(.a(new_n97), .b(new_n98), .out0(new_n126));
  nanb02aa1n02x5               g031(.a(new_n99), .b(new_n100), .out0(new_n127));
  norp02aa1n02x5               g032(.a(new_n105), .b(new_n104), .o1(new_n128));
  nona22aa1n02x4               g033(.a(new_n128), .b(new_n127), .c(new_n126), .out0(new_n129));
  160nm_ficinv00aa1n08x5       g034(.clk(new_n110), .clkout(new_n130));
  nano23aa1n02x4               g035(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n131));
  aobi12aa1n02x5               g036(.a(new_n116), .b(new_n131), .c(new_n130), .out0(new_n132));
  norp03aa1n02x5               g037(.a(new_n121), .b(new_n127), .c(new_n126), .o1(new_n133));
  nano22aa1n02x4               g038(.a(new_n133), .b(new_n118), .c(new_n119), .out0(new_n134));
  oai012aa1n02x5               g039(.a(new_n134), .b(new_n132), .c(new_n129), .o1(new_n135));
  norp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(\a[10] ), .clkout(new_n139));
  oai022aa1n02x5               g044(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n140));
  oaib12aa1n02x5               g045(.a(new_n140), .b(new_n139), .c(\b[9] ), .out0(new_n141));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n141), .clkout(new_n142));
  160nm_ficinv00aa1n08x5       g047(.clk(\a[9] ), .clkout(new_n143));
  xroi22aa1d04x5               g048(.a(\b[8] ), .b(new_n143), .c(new_n139), .d(\b[9] ), .out0(new_n144));
  aoai13aa1n02x5               g049(.a(new_n138), .b(new_n142), .c(new_n135), .d(new_n144), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(new_n142), .b(new_n138), .c(new_n135), .d(new_n144), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n136), .clkout(new_n148));
  norp02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n145), .c(new_n148), .out0(\s[12] ));
  xorc02aa1n02x5               g057(.a(\a[10] ), .b(\b[9] ), .out0(new_n153));
  xorc02aa1n02x5               g058(.a(\a[9] ), .b(\b[8] ), .out0(new_n154));
  nano23aa1n02x4               g059(.a(new_n136), .b(new_n149), .c(new_n150), .d(new_n137), .out0(new_n155));
  nanp03aa1n02x5               g060(.a(new_n155), .b(new_n153), .c(new_n154), .o1(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n156), .clkout(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n122), .c(new_n117), .d(new_n106), .o1(new_n158));
  nona23aa1n02x4               g063(.a(new_n150), .b(new_n137), .c(new_n136), .d(new_n149), .out0(new_n159));
  aoi012aa1n02x5               g064(.a(new_n149), .b(new_n136), .c(new_n150), .o1(new_n160));
  oai012aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n141), .o1(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  norp02aa1n02x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  xobna2aa1n03x5               g070(.a(new_n165), .b(new_n158), .c(new_n162), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n163), .clkout(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n165), .c(new_n158), .d(new_n162), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nona23aa1n02x4               g076(.a(new_n171), .b(new_n164), .c(new_n163), .d(new_n170), .out0(new_n172));
  oaoi03aa1n02x5               g077(.a(\a[14] ), .b(\b[13] ), .c(new_n167), .o1(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n173), .clkout(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n172), .c(new_n158), .d(new_n162), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  xorc02aa1n02x5               g082(.a(\a[15] ), .b(\b[14] ), .out0(new_n178));
  xorc02aa1n02x5               g083(.a(\a[16] ), .b(\b[15] ), .out0(new_n179));
  aoi112aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n175), .d(new_n178), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n179), .b(new_n177), .c(new_n175), .d(new_n178), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n181), .b(new_n180), .out0(\s[16] ));
  xnrc02aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .out0(new_n183));
  xnrc02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .out0(new_n184));
  norp03aa1n02x5               g089(.a(new_n172), .b(new_n184), .c(new_n183), .o1(new_n185));
  nanp03aa1n02x5               g090(.a(new_n185), .b(new_n144), .c(new_n155), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(new_n179), .b(new_n178), .o1(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(new_n177), .clkout(new_n188));
  oaoi03aa1n02x5               g093(.a(\a[16] ), .b(\b[15] ), .c(new_n188), .o1(new_n189));
  oabi12aa1n02x5               g094(.a(new_n189), .b(new_n187), .c(new_n174), .out0(new_n190));
  aoi012aa1n02x5               g095(.a(new_n190), .b(new_n161), .c(new_n185), .o1(new_n191));
  oai012aa1n02x5               g096(.a(new_n191), .b(new_n123), .c(new_n186), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g098(.clk(\a[18] ), .clkout(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(\a[17] ), .clkout(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(\b[16] ), .clkout(new_n196));
  oaoi03aa1n02x5               g101(.a(new_n195), .b(new_n196), .c(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n194), .out0(\s[18] ));
  norp03aa1n02x5               g103(.a(new_n156), .b(new_n172), .c(new_n187), .o1(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n122), .c(new_n106), .d(new_n117), .o1(new_n200));
  xroi22aa1d04x5               g105(.a(new_n195), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n201), .clkout(new_n202));
  oai022aa1n02x5               g107(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n203));
  oaib12aa1n02x5               g108(.a(new_n203), .b(new_n194), .c(\b[17] ), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n202), .c(new_n200), .d(new_n191), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(\b[19] ), .clkout(new_n212));
  nanb02aa1n02x5               g117(.a(\a[20] ), .b(new_n212), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(new_n213), .b(new_n214), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  aoi112aa1n02x5               g121(.a(new_n208), .b(new_n216), .c(new_n205), .d(new_n211), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n208), .clkout(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n204), .clkout(new_n219));
  aoai13aa1n02x5               g124(.a(new_n211), .b(new_n219), .c(new_n192), .d(new_n201), .o1(new_n220));
  aoi012aa1n02x5               g125(.a(new_n215), .b(new_n220), .c(new_n218), .o1(new_n221));
  norp02aa1n02x5               g126(.a(new_n221), .b(new_n217), .o1(\s[20] ));
  norp02aa1n02x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nona23aa1n02x4               g128(.a(new_n214), .b(new_n209), .c(new_n208), .d(new_n223), .out0(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n224), .clkout(new_n225));
  nanp02aa1n02x5               g130(.a(new_n201), .b(new_n225), .o1(new_n226));
  nanp02aa1n02x5               g131(.a(new_n208), .b(new_n214), .o1(new_n227));
  norp03aa1n02x5               g132(.a(new_n204), .b(new_n210), .c(new_n215), .o1(new_n228));
  nano22aa1n02x4               g133(.a(new_n228), .b(new_n213), .c(new_n227), .out0(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n226), .c(new_n200), .d(new_n191), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xnrc02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .out0(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  xnrc02aa1n02x5               g139(.a(\b[21] ), .b(\a[22] ), .out0(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoi112aa1n02x5               g141(.a(new_n232), .b(new_n236), .c(new_n230), .d(new_n234), .o1(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n232), .clkout(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n226), .clkout(new_n239));
  oai112aa1n02x5               g144(.a(new_n227), .b(new_n213), .c(new_n224), .d(new_n204), .o1(new_n240));
  aoai13aa1n02x5               g145(.a(new_n234), .b(new_n240), .c(new_n192), .d(new_n239), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n235), .b(new_n241), .c(new_n238), .o1(new_n242));
  norp02aa1n02x5               g147(.a(new_n242), .b(new_n237), .o1(\s[22] ));
  norp02aa1n02x5               g148(.a(new_n235), .b(new_n233), .o1(new_n244));
  oaoi03aa1n02x5               g149(.a(\a[22] ), .b(\b[21] ), .c(new_n238), .o1(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n240), .c(new_n244), .o1(new_n246));
  nanp03aa1n02x5               g151(.a(new_n201), .b(new_n225), .c(new_n244), .o1(new_n247));
  aoai13aa1n02x5               g152(.a(new_n246), .b(new_n247), .c(new_n200), .d(new_n191), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[23] ), .b(\b[22] ), .out0(new_n251));
  xorc02aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .out0(new_n252));
  aoi112aa1n02x5               g157(.a(new_n250), .b(new_n252), .c(new_n248), .d(new_n251), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n250), .clkout(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n246), .clkout(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n247), .clkout(new_n256));
  aoai13aa1n02x5               g161(.a(new_n251), .b(new_n255), .c(new_n192), .d(new_n256), .o1(new_n257));
  aobi12aa1n02x5               g162(.a(new_n252), .b(new_n257), .c(new_n254), .out0(new_n258));
  norp02aa1n02x5               g163(.a(new_n258), .b(new_n253), .o1(\s[24] ));
  nano32aa1n02x4               g164(.a(new_n226), .b(new_n252), .c(new_n244), .d(new_n251), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n260), .clkout(new_n261));
  xnrc02aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .out0(new_n262));
  norb02aa1n02x5               g167(.a(new_n252), .b(new_n262), .out0(new_n263));
  norp02aa1n02x5               g168(.a(\b[23] ), .b(\a[24] ), .o1(new_n264));
  aoi112aa1n02x5               g169(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n265));
  nanp03aa1n02x5               g170(.a(new_n245), .b(new_n251), .c(new_n252), .o1(new_n266));
  nona22aa1n02x4               g171(.a(new_n266), .b(new_n265), .c(new_n264), .out0(new_n267));
  aoi013aa1n02x4               g172(.a(new_n267), .b(new_n240), .c(new_n244), .d(new_n263), .o1(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n261), .c(new_n200), .d(new_n191), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g175(.a(\b[24] ), .b(\a[25] ), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  xorc02aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .out0(new_n273));
  aoi112aa1n02x5               g178(.a(new_n271), .b(new_n273), .c(new_n269), .d(new_n272), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n271), .clkout(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n268), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n272), .b(new_n276), .c(new_n192), .d(new_n260), .o1(new_n277));
  aobi12aa1n02x5               g182(.a(new_n273), .b(new_n277), .c(new_n275), .out0(new_n278));
  norp02aa1n02x5               g183(.a(new_n278), .b(new_n274), .o1(\s[26] ));
  aoi013aa1n02x4               g184(.a(new_n189), .b(new_n173), .c(new_n179), .d(new_n178), .o1(new_n280));
  aob012aa1n02x5               g185(.a(new_n280), .b(new_n161), .c(new_n185), .out0(new_n281));
  and002aa1n02x5               g186(.a(new_n273), .b(new_n272), .o(new_n282));
  nano22aa1n02x4               g187(.a(new_n247), .b(new_n282), .c(new_n263), .out0(new_n283));
  aoai13aa1n02x5               g188(.a(new_n283), .b(new_n281), .c(new_n135), .d(new_n199), .o1(new_n284));
  nano22aa1n02x4               g189(.a(new_n229), .b(new_n244), .c(new_n263), .out0(new_n285));
  oao003aa1n02x5               g190(.a(\a[26] ), .b(\b[25] ), .c(new_n275), .carry(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n286), .clkout(new_n287));
  oaoi13aa1n02x5               g192(.a(new_n287), .b(new_n282), .c(new_n285), .d(new_n267), .o1(new_n288));
  norp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  nanp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  norb02aa1n02x5               g195(.a(new_n290), .b(new_n289), .out0(new_n291));
  xnbna2aa1n03x5               g196(.a(new_n291), .b(new_n288), .c(new_n284), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g197(.clk(new_n289), .clkout(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[27] ), .b(\a[28] ), .out0(new_n294));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n267), .clkout(new_n295));
  nanp02aa1n02x5               g200(.a(new_n252), .b(new_n251), .o1(new_n296));
  nona32aa1n02x4               g201(.a(new_n240), .b(new_n296), .c(new_n235), .d(new_n233), .out0(new_n297));
  160nm_ficinv00aa1n08x5       g202(.clk(new_n282), .clkout(new_n298));
  aoai13aa1n02x5               g203(.a(new_n286), .b(new_n298), .c(new_n297), .d(new_n295), .o1(new_n299));
  aoai13aa1n02x5               g204(.a(new_n290), .b(new_n299), .c(new_n192), .d(new_n283), .o1(new_n300));
  aoi012aa1n02x5               g205(.a(new_n294), .b(new_n300), .c(new_n293), .o1(new_n301));
  aoi022aa1n02x5               g206(.a(new_n288), .b(new_n284), .c(\a[27] ), .d(\b[26] ), .o1(new_n302));
  nano22aa1n02x4               g207(.a(new_n302), .b(new_n293), .c(new_n294), .out0(new_n303));
  norp02aa1n02x5               g208(.a(new_n301), .b(new_n303), .o1(\s[28] ));
  nano22aa1n02x4               g209(.a(new_n294), .b(new_n293), .c(new_n290), .out0(new_n305));
  aoai13aa1n02x5               g210(.a(new_n305), .b(new_n299), .c(new_n192), .d(new_n283), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[28] ), .b(\a[29] ), .out0(new_n308));
  aoi012aa1n02x5               g213(.a(new_n308), .b(new_n306), .c(new_n307), .o1(new_n309));
  aobi12aa1n02x5               g214(.a(new_n305), .b(new_n288), .c(new_n284), .out0(new_n310));
  nano22aa1n02x4               g215(.a(new_n310), .b(new_n307), .c(new_n308), .out0(new_n311));
  norp02aa1n02x5               g216(.a(new_n309), .b(new_n311), .o1(\s[29] ));
  xorb03aa1n02x5               g217(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g218(.a(new_n291), .b(new_n308), .c(new_n294), .out0(new_n314));
  aoai13aa1n02x5               g219(.a(new_n314), .b(new_n299), .c(new_n192), .d(new_n283), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .carry(new_n316));
  xnrc02aa1n02x5               g221(.a(\b[29] ), .b(\a[30] ), .out0(new_n317));
  aoi012aa1n02x5               g222(.a(new_n317), .b(new_n315), .c(new_n316), .o1(new_n318));
  aobi12aa1n02x5               g223(.a(new_n314), .b(new_n288), .c(new_n284), .out0(new_n319));
  nano22aa1n02x4               g224(.a(new_n319), .b(new_n316), .c(new_n317), .out0(new_n320));
  norp02aa1n02x5               g225(.a(new_n318), .b(new_n320), .o1(\s[30] ));
  xnrc02aa1n02x5               g226(.a(\b[30] ), .b(\a[31] ), .out0(new_n322));
  norb03aa1n02x5               g227(.a(new_n305), .b(new_n317), .c(new_n308), .out0(new_n323));
  aobi12aa1n02x5               g228(.a(new_n323), .b(new_n288), .c(new_n284), .out0(new_n324));
  oao003aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .c(new_n316), .carry(new_n325));
  nano22aa1n02x4               g230(.a(new_n324), .b(new_n322), .c(new_n325), .out0(new_n326));
  aoai13aa1n02x5               g231(.a(new_n323), .b(new_n299), .c(new_n192), .d(new_n283), .o1(new_n327));
  aoi012aa1n02x5               g232(.a(new_n322), .b(new_n327), .c(new_n325), .o1(new_n328));
  norp02aa1n02x5               g233(.a(new_n328), .b(new_n326), .o1(\s[31] ));
  xnrb03aa1n02x5               g234(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g235(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g237(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g238(.a(\a[5] ), .b(\b[4] ), .c(new_n132), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aobi12aa1n02x5               g240(.a(new_n121), .b(new_n117), .c(new_n128), .out0(new_n336));
  xnrb03aa1n02x5               g241(.a(new_n336), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g242(.a(\a[7] ), .b(\b[6] ), .c(new_n336), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g244(.a(new_n123), .b(\b[8] ), .c(new_n143), .out0(\s[9] ));
endmodule


