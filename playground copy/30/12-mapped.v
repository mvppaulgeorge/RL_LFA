// Benchmark "adder" written by ABC on Thu Jul 11 13:05:45 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n323, new_n324;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\a[2] ), .clkout(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\b[1] ), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oao003aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n02x4               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  aobi12aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .out0(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n02x4               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nona22aa1n02x4               g021(.a(new_n114), .b(new_n115), .c(new_n116), .out0(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[6] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[5] ), .clkout(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  oao003aa1n02x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .carry(new_n121));
  160nm_fiao0012aa1n02p5x5     g026(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .o1(new_n123));
  oai012aa1n02x5               g028(.a(new_n123), .b(new_n109), .c(new_n117), .o1(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  160nm_ficinv00aa1n08x5       g030(.clk(new_n125), .clkout(new_n126));
  aoi012aa1n02x5               g031(.a(new_n98), .b(new_n124), .c(new_n126), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  oaoi03aa1n02x5               g033(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n129));
  nona23aa1n02x4               g034(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n130));
  oai012aa1n02x5               g035(.a(new_n108), .b(new_n130), .c(new_n129), .o1(new_n131));
  nona23aa1n02x4               g036(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n132));
  norp03aa1n02x5               g037(.a(new_n132), .b(new_n115), .c(new_n116), .o1(new_n133));
  oaoi03aa1n02x5               g038(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n134));
  oabi12aa1n02x5               g039(.a(new_n122), .b(new_n132), .c(new_n134), .out0(new_n135));
  xnrc02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .out0(new_n136));
  norp02aa1n02x5               g041(.a(new_n125), .b(new_n136), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n135), .c(new_n131), .d(new_n133), .o1(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(\b[9] ), .clkout(new_n139));
  oaoi03aa1n02x5               g044(.a(new_n97), .b(new_n139), .c(new_n98), .o1(new_n140));
  norp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n138), .c(new_n140), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g049(.clk(new_n141), .clkout(new_n145));
  aob012aa1n02x5               g050(.a(new_n143), .b(new_n138), .c(new_n140), .out0(new_n146));
  norp02aa1n02x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n146), .c(new_n145), .out0(\s[12] ));
  nano23aa1n02x4               g055(.a(new_n141), .b(new_n147), .c(new_n148), .d(new_n142), .out0(new_n151));
  nona23aa1n02x4               g056(.a(new_n148), .b(new_n142), .c(new_n141), .d(new_n147), .out0(new_n152));
  aoi012aa1n02x5               g057(.a(new_n147), .b(new_n141), .c(new_n148), .o1(new_n153));
  oai012aa1n02x5               g058(.a(new_n153), .b(new_n152), .c(new_n140), .o1(new_n154));
  aoi013aa1n02x4               g059(.a(new_n154), .b(new_n124), .c(new_n137), .d(new_n151), .o1(new_n155));
  norp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n156), .clkout(new_n157));
  nanp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n155), .b(new_n158), .c(new_n157), .out0(\s[13] ));
  oaoi03aa1n02x5               g064(.a(\a[13] ), .b(\b[12] ), .c(new_n155), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n137), .clkout(new_n162));
  norp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nona23aa1n02x4               g069(.a(new_n164), .b(new_n158), .c(new_n156), .d(new_n163), .out0(new_n165));
  nona32aa1n02x4               g070(.a(new_n124), .b(new_n165), .c(new_n152), .d(new_n162), .out0(new_n166));
  nano23aa1n02x4               g071(.a(new_n156), .b(new_n163), .c(new_n164), .d(new_n158), .out0(new_n167));
  oaoi03aa1n02x5               g072(.a(\a[14] ), .b(\b[13] ), .c(new_n157), .o1(new_n168));
  aoi012aa1n02x5               g073(.a(new_n168), .b(new_n154), .c(new_n167), .o1(new_n169));
  xorc02aa1n02x5               g074(.a(\a[15] ), .b(\b[14] ), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n166), .c(new_n169), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g076(.clk(\a[16] ), .clkout(new_n172));
  nanp02aa1n02x5               g077(.a(new_n166), .b(new_n169), .o1(new_n173));
  norp02aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  aoi012aa1n02x5               g079(.a(new_n174), .b(new_n173), .c(new_n170), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[15] ), .c(new_n172), .out0(\s[16] ));
  xorc02aa1n02x5               g081(.a(\a[16] ), .b(\b[15] ), .out0(new_n177));
  nanp03aa1n02x5               g082(.a(new_n167), .b(new_n170), .c(new_n177), .o1(new_n178));
  nano22aa1n02x4               g083(.a(new_n178), .b(new_n137), .c(new_n151), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n135), .c(new_n131), .d(new_n133), .o1(new_n180));
  xnrc02aa1n02x5               g085(.a(\b[14] ), .b(\a[15] ), .out0(new_n181));
  xnrc02aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .out0(new_n182));
  norp03aa1n02x5               g087(.a(new_n165), .b(new_n181), .c(new_n182), .o1(new_n183));
  and003aa1n02x5               g088(.a(new_n168), .b(new_n177), .c(new_n170), .o(new_n184));
  aob012aa1n02x5               g089(.a(new_n174), .b(\b[15] ), .c(\a[16] ), .out0(new_n185));
  oaib12aa1n02x5               g090(.a(new_n185), .b(\b[15] ), .c(new_n172), .out0(new_n186));
  aoi112aa1n02x5               g091(.a(new_n186), .b(new_n184), .c(new_n154), .d(new_n183), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(new_n180), .b(new_n187), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[18] ), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\a[17] ), .clkout(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(\b[16] ), .clkout(new_n192));
  oaoi03aa1n02x5               g097(.a(new_n191), .b(new_n192), .c(new_n188), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[17] ), .c(new_n190), .out0(\s[18] ));
  xroi22aa1d04x5               g099(.a(new_n191), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n195));
  norp02aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n197));
  norp02aa1n02x5               g102(.a(new_n197), .b(new_n196), .o1(new_n198));
  160nm_ficinv00aa1n08x5       g103(.clk(new_n198), .clkout(new_n199));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n199), .c(new_n188), .d(new_n195), .o1(new_n203));
  aoi112aa1n02x5               g108(.a(new_n202), .b(new_n199), .c(new_n188), .d(new_n195), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  nona22aa1n02x4               g114(.a(new_n203), .b(new_n209), .c(new_n200), .out0(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n200), .clkout(new_n211));
  aobi12aa1n02x5               g116(.a(new_n209), .b(new_n203), .c(new_n211), .out0(new_n212));
  norb02aa1n02x5               g117(.a(new_n210), .b(new_n212), .out0(\s[20] ));
  nona23aa1n02x4               g118(.a(new_n208), .b(new_n201), .c(new_n200), .d(new_n207), .out0(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  nanp02aa1n02x5               g120(.a(new_n195), .b(new_n215), .o1(new_n216));
  aoi012aa1n02x5               g121(.a(new_n207), .b(new_n200), .c(new_n208), .o1(new_n217));
  oai012aa1n02x5               g122(.a(new_n217), .b(new_n214), .c(new_n198), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n216), .c(new_n180), .d(new_n187), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[21] ), .b(\b[20] ), .out0(new_n223));
  xorc02aa1n02x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n222), .b(new_n224), .c(new_n220), .d(new_n223), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n224), .b(new_n222), .c(new_n220), .d(new_n223), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(\s[22] ));
  oai112aa1n02x5               g132(.a(new_n202), .b(new_n209), .c(new_n197), .d(new_n196), .o1(new_n228));
  nanp02aa1n02x5               g133(.a(new_n224), .b(new_n223), .o1(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[22] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\b[21] ), .clkout(new_n231));
  oaoi03aa1n02x5               g136(.a(new_n230), .b(new_n231), .c(new_n222), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n229), .c(new_n228), .d(new_n217), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  nanb03aa1n02x5               g139(.a(new_n229), .b(new_n195), .c(new_n215), .out0(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n235), .c(new_n180), .d(new_n187), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoi112aa1n02x5               g146(.a(new_n238), .b(new_n241), .c(new_n236), .d(new_n240), .o1(new_n242));
  aoai13aa1n02x5               g147(.a(new_n241), .b(new_n238), .c(new_n236), .d(new_n239), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  and002aa1n02x5               g149(.a(new_n241), .b(new_n240), .o(new_n245));
  nona23aa1n02x4               g150(.a(new_n195), .b(new_n245), .c(new_n229), .d(new_n214), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(\a[24] ), .clkout(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(\b[23] ), .clkout(new_n248));
  oaoi03aa1n02x5               g153(.a(new_n247), .b(new_n248), .c(new_n238), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  aoi012aa1n02x5               g155(.a(new_n250), .b(new_n233), .c(new_n245), .o1(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n246), .c(new_n180), .d(new_n187), .o1(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  xorc02aa1n02x5               g160(.a(\a[26] ), .b(\b[25] ), .out0(new_n256));
  aoi112aa1n02x5               g161(.a(new_n254), .b(new_n256), .c(new_n252), .d(new_n255), .o1(new_n257));
  aoai13aa1n02x5               g162(.a(new_n256), .b(new_n254), .c(new_n252), .d(new_n255), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n258), .b(new_n257), .out0(\s[26] ));
  aoi013aa1n02x4               g164(.a(new_n186), .b(new_n168), .c(new_n170), .d(new_n177), .o1(new_n260));
  aob012aa1n02x5               g165(.a(new_n260), .b(new_n154), .c(new_n183), .out0(new_n261));
  and002aa1n02x5               g166(.a(new_n256), .b(new_n255), .o(new_n262));
  nano22aa1n02x4               g167(.a(new_n235), .b(new_n245), .c(new_n262), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n261), .c(new_n124), .d(new_n179), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n262), .b(new_n250), .c(new_n233), .d(new_n245), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n266));
  oab012aa1n02x4               g171(.a(new_n266), .b(\a[26] ), .c(\b[25] ), .out0(new_n267));
  nanp03aa1n02x5               g172(.a(new_n264), .b(new_n265), .c(new_n267), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n263), .clkout(new_n273));
  aoi012aa1n02x5               g178(.a(new_n273), .b(new_n180), .c(new_n187), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n229), .clkout(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n232), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n245), .b(new_n276), .c(new_n218), .d(new_n275), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n262), .clkout(new_n278));
  aoai13aa1n02x5               g183(.a(new_n267), .b(new_n278), .c(new_n277), .d(new_n249), .o1(new_n279));
  and002aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n280), .clkout(new_n281));
  oai012aa1n02x5               g186(.a(new_n281), .b(new_n279), .c(new_n274), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n272), .b(new_n282), .c(new_n271), .o1(new_n283));
  aoi013aa1n02x4               g188(.a(new_n280), .b(new_n264), .c(new_n265), .d(new_n267), .o1(new_n284));
  nano22aa1n02x4               g189(.a(new_n284), .b(new_n271), .c(new_n272), .out0(new_n285));
  norp02aa1n02x5               g190(.a(new_n283), .b(new_n285), .o1(\s[28] ));
  nano22aa1n02x4               g191(.a(new_n272), .b(new_n281), .c(new_n271), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n287), .b(new_n279), .c(new_n274), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n287), .clkout(new_n292));
  aoi013aa1n02x4               g197(.a(new_n292), .b(new_n264), .c(new_n265), .d(new_n267), .o1(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n289), .c(new_n290), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n291), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g201(.a(new_n290), .b(new_n272), .c(new_n281), .d(new_n271), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n297), .b(new_n279), .c(new_n274), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n297), .clkout(new_n302));
  aoi013aa1n02x4               g207(.a(new_n302), .b(new_n264), .c(new_n265), .d(new_n267), .o1(new_n303));
  nano22aa1n02x4               g208(.a(new_n303), .b(new_n299), .c(new_n300), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n301), .b(new_n304), .o1(\s[30] ));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  norb03aa1n02x5               g211(.a(new_n287), .b(new_n300), .c(new_n290), .out0(new_n307));
  160nm_ficinv00aa1n08x5       g212(.clk(new_n307), .clkout(new_n308));
  aoi013aa1n02x4               g213(.a(new_n308), .b(new_n264), .c(new_n265), .d(new_n267), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n310));
  nano22aa1n02x4               g215(.a(new_n309), .b(new_n306), .c(new_n310), .out0(new_n311));
  oai012aa1n02x5               g216(.a(new_n307), .b(new_n279), .c(new_n274), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n306), .b(new_n312), .c(new_n310), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g219(.a(new_n129), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n129), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n131), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g223(.a(\a[5] ), .b(\b[4] ), .c(new_n109), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g225(.a(new_n118), .b(new_n119), .c(new_n319), .carry(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  160nm_ficinv00aa1n08x5       g227(.clk(\a[8] ), .clkout(new_n323));
  aoi012aa1n02x5               g228(.a(new_n112), .b(new_n321), .c(new_n113), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(new_n323), .out0(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


