// Benchmark "adder" written by ABC on Thu Jul 11 12:09:36 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n322, new_n324, new_n325;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[2] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[1] ), .clkout(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  oao003aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .carry(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano23aa1n02x4               g010(.a(new_n102), .b(new_n104), .c(new_n105), .d(new_n103), .out0(new_n106));
  oai012aa1n02x5               g011(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n107));
  aobi12aa1n02x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .out0(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n02x4               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  nona22aa1n02x4               g020(.a(new_n113), .b(new_n114), .c(new_n115), .out0(new_n116));
  orn002aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .o(new_n117));
  oaoi03aa1n02x5               g022(.a(\a[6] ), .b(\b[5] ), .c(new_n117), .o1(new_n118));
  oai012aa1n02x5               g023(.a(new_n110), .b(new_n111), .c(new_n109), .o1(new_n119));
  aobi12aa1n02x5               g024(.a(new_n119), .b(new_n113), .c(new_n118), .out0(new_n120));
  oai012aa1n02x5               g025(.a(new_n120), .b(new_n108), .c(new_n116), .o1(new_n121));
  xorc02aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  xorc02aa1n02x5               g027(.a(\a[10] ), .b(\b[9] ), .out0(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(new_n123), .clkout(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n97), .c(new_n121), .d(new_n122), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n126));
  nona23aa1n02x4               g031(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n127));
  oai012aa1n02x5               g032(.a(new_n107), .b(new_n127), .c(new_n126), .o1(new_n128));
  nona23aa1n02x4               g033(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n129));
  norp03aa1n02x5               g034(.a(new_n129), .b(new_n114), .c(new_n115), .o1(new_n130));
  oaib12aa1n02x5               g035(.a(new_n119), .b(new_n129), .c(new_n118), .out0(new_n131));
  aoai13aa1n02x5               g036(.a(new_n122), .b(new_n131), .c(new_n128), .d(new_n130), .o1(new_n132));
  nona22aa1n02x4               g037(.a(new_n132), .b(new_n124), .c(new_n97), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n125), .b(new_n133), .o1(\s[10] ));
  160nm_ficinv00aa1n08x5       g039(.clk(\a[10] ), .clkout(new_n135));
  160nm_ficinv00aa1n08x5       g040(.clk(\b[9] ), .clkout(new_n136));
  norp02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  oai112aa1n02x5               g044(.a(new_n133), .b(new_n139), .c(new_n136), .d(new_n135), .o1(new_n140));
  oaoi13aa1n02x5               g045(.a(new_n139), .b(new_n133), .c(new_n135), .d(new_n136), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(\s[11] ));
  orn002aa1n02x5               g047(.a(\a[11] ), .b(\b[10] ), .o(new_n143));
  norp02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n140), .c(new_n143), .out0(\s[12] ));
  nanp02aa1n02x5               g052(.a(new_n128), .b(new_n130), .o1(new_n148));
  nano23aa1n02x4               g053(.a(new_n137), .b(new_n144), .c(new_n145), .d(new_n138), .out0(new_n149));
  nanp03aa1n02x5               g054(.a(new_n149), .b(new_n122), .c(new_n123), .o1(new_n150));
  nona23aa1n02x4               g055(.a(new_n145), .b(new_n138), .c(new_n137), .d(new_n144), .out0(new_n151));
  oaoi03aa1n02x5               g056(.a(new_n135), .b(new_n136), .c(new_n97), .o1(new_n152));
  oai012aa1n02x5               g057(.a(new_n145), .b(new_n144), .c(new_n137), .o1(new_n153));
  oai012aa1n02x5               g058(.a(new_n153), .b(new_n151), .c(new_n152), .o1(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n150), .c(new_n148), .d(new_n120), .o1(new_n156));
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
  norp02aa1n02x5               g087(.a(new_n182), .b(new_n150), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n131), .c(new_n128), .d(new_n130), .o1(new_n184));
  oai012aa1n02x5               g089(.a(new_n175), .b(new_n174), .c(new_n163), .o1(new_n185));
  oaib12aa1n02x5               g090(.a(new_n185), .b(new_n169), .c(new_n181), .out0(new_n186));
  aoib12aa1n02x5               g091(.a(new_n186), .b(new_n154), .c(new_n182), .out0(new_n187));
  nanp02aa1n02x5               g092(.a(new_n184), .b(new_n187), .o1(new_n188));
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
  oai012aa1n02x5               g121(.a(new_n208), .b(new_n207), .c(new_n200), .o1(new_n217));
  oai012aa1n02x5               g122(.a(new_n217), .b(new_n214), .c(new_n198), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n216), .c(new_n184), .d(new_n187), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[21] ), .b(\b[20] ), .out0(new_n223));
  xorc02aa1n02x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n222), .b(new_n224), .c(new_n220), .d(new_n223), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n224), .b(new_n222), .c(new_n220), .d(new_n223), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(\s[22] ));
  nanp02aa1n02x5               g132(.a(new_n224), .b(new_n223), .o1(new_n228));
  nanb03aa1n02x5               g133(.a(new_n228), .b(new_n195), .c(new_n215), .out0(new_n229));
  oai112aa1n02x5               g134(.a(new_n202), .b(new_n209), .c(new_n197), .d(new_n196), .o1(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\a[22] ), .clkout(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(\b[21] ), .clkout(new_n232));
  oao003aa1n02x5               g137(.a(new_n231), .b(new_n232), .c(new_n222), .carry(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n228), .c(new_n230), .d(new_n217), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n229), .c(new_n184), .d(new_n187), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoi112aa1n02x5               g146(.a(new_n239), .b(new_n241), .c(new_n237), .d(new_n240), .o1(new_n242));
  aoai13aa1n02x5               g147(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  and002aa1n02x5               g149(.a(new_n241), .b(new_n240), .o(new_n245));
  nona23aa1n02x4               g150(.a(new_n245), .b(new_n195), .c(new_n228), .d(new_n214), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(\a[24] ), .clkout(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(\b[23] ), .clkout(new_n248));
  oao003aa1n02x5               g153(.a(new_n247), .b(new_n248), .c(new_n239), .carry(new_n249));
  aoi012aa1n02x5               g154(.a(new_n249), .b(new_n235), .c(new_n245), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n246), .c(new_n184), .d(new_n187), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xorc02aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  aoi112aa1n02x5               g160(.a(new_n253), .b(new_n255), .c(new_n251), .d(new_n254), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n255), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n257), .b(new_n256), .out0(\s[26] ));
  nanb02aa1n02x5               g163(.a(new_n152), .b(new_n149), .out0(new_n259));
  aobi12aa1n02x5               g164(.a(new_n185), .b(new_n170), .c(new_n181), .out0(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n182), .c(new_n259), .d(new_n153), .o1(new_n261));
  and002aa1n02x5               g166(.a(new_n255), .b(new_n254), .o(new_n262));
  nano22aa1n02x4               g167(.a(new_n229), .b(new_n245), .c(new_n262), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n261), .c(new_n121), .d(new_n183), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n262), .b(new_n249), .c(new_n235), .d(new_n245), .o1(new_n265));
  oai022aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n266));
  aob012aa1n02x5               g171(.a(new_n266), .b(\b[25] ), .c(\a[26] ), .out0(new_n267));
  xorc02aa1n02x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n268), .clkout(new_n269));
  aoi013aa1n02x4               g174(.a(new_n269), .b(new_n264), .c(new_n265), .d(new_n267), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n263), .clkout(new_n271));
  aoi012aa1n02x5               g176(.a(new_n271), .b(new_n184), .c(new_n187), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n228), .clkout(new_n273));
  aoai13aa1n02x5               g178(.a(new_n245), .b(new_n233), .c(new_n218), .d(new_n273), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n249), .clkout(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n262), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n267), .b(new_n276), .c(new_n274), .d(new_n275), .o1(new_n277));
  norp03aa1n02x5               g182(.a(new_n277), .b(new_n272), .c(new_n268), .o1(new_n278));
  norp02aa1n02x5               g183(.a(new_n270), .b(new_n278), .o1(\s[27] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n280), .clkout(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  nano22aa1n02x4               g187(.a(new_n270), .b(new_n281), .c(new_n282), .out0(new_n283));
  oai012aa1n02x5               g188(.a(new_n268), .b(new_n277), .c(new_n272), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n282), .b(new_n284), .c(new_n281), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n285), .b(new_n283), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n268), .b(new_n282), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n287), .b(new_n277), .c(new_n272), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n287), .clkout(new_n292));
  aoi013aa1n02x4               g197(.a(new_n292), .b(new_n264), .c(new_n265), .d(new_n267), .o1(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n289), .c(new_n290), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n291), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n268), .b(new_n290), .c(new_n282), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n297), .b(new_n277), .c(new_n272), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n297), .clkout(new_n302));
  aoi013aa1n02x4               g207(.a(new_n302), .b(new_n264), .c(new_n265), .d(new_n267), .o1(new_n303));
  nano22aa1n02x4               g208(.a(new_n303), .b(new_n299), .c(new_n300), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n301), .b(new_n304), .o1(\s[30] ));
  norb02aa1n02x5               g210(.a(new_n297), .b(new_n300), .out0(new_n306));
  160nm_ficinv00aa1n08x5       g211(.clk(new_n306), .clkout(new_n307));
  aoi013aa1n02x4               g212(.a(new_n307), .b(new_n264), .c(new_n265), .d(new_n267), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nano22aa1n02x4               g215(.a(new_n308), .b(new_n309), .c(new_n310), .out0(new_n311));
  oai012aa1n02x5               g216(.a(new_n306), .b(new_n277), .c(new_n272), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n310), .b(new_n312), .c(new_n309), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g219(.a(new_n126), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n126), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n128), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g223(.a(\a[5] ), .b(\b[4] ), .c(new_n108), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g225(.a(\a[5] ), .b(\b[4] ), .c(new_n108), .carry(new_n321));
  oaoi03aa1n02x5               g226(.a(\a[6] ), .b(\b[5] ), .c(new_n321), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  160nm_ficinv00aa1n08x5       g228(.clk(\a[8] ), .clkout(new_n324));
  aoi012aa1n02x5               g229(.a(new_n111), .b(new_n322), .c(new_n112), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[7] ), .c(new_n324), .out0(\s[8] ));
  xnbna2aa1n03x5               g231(.a(new_n122), .b(new_n148), .c(new_n120), .out0(\s[9] ));
endmodule


