// Benchmark "adder" written by ABC on Wed Jul 10 16:56:28 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n320,
    new_n321, new_n323;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  and002aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(new_n99), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\a[4] ), .clkout(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\a[3] ), .clkout(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\b[2] ), .clkout(new_n103));
  nanp02aa1n02x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  aoi012aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  aboi22aa1n03x5               g015(.a(\b[3] ), .b(new_n101), .c(new_n102), .d(new_n103), .out0(new_n111));
  oai012aa1n02x5               g016(.a(new_n111), .b(new_n110), .c(new_n106), .o1(new_n112));
  oaib12aa1n02x5               g017(.a(new_n112), .b(new_n101), .c(\b[3] ), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[6] ), .b(\b[5] ), .out0(new_n114));
  norp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nanb02aa1n02x5               g021(.a(new_n115), .b(new_n116), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .out0(new_n118));
  xorc02aa1n02x5               g023(.a(\a[5] ), .b(\b[4] ), .out0(new_n119));
  nona23aa1n02x4               g024(.a(new_n114), .b(new_n119), .c(new_n118), .d(new_n117), .out0(new_n120));
  aoi112aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  oai022aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  nano23aa1n02x4               g028(.a(new_n118), .b(new_n117), .c(new_n123), .d(new_n122), .out0(new_n124));
  norp03aa1n02x5               g029(.a(new_n124), .b(new_n121), .c(new_n115), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  oai112aa1n02x5               g031(.a(new_n125), .b(new_n126), .c(new_n113), .d(new_n120), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n98), .b(new_n127), .c(new_n100), .out0(\s[10] ));
  nanp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  aob012aa1n02x5               g034(.a(new_n97), .b(new_n127), .c(new_n100), .out0(new_n130));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xobna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n129), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n131), .clkout(new_n135));
  nano22aa1n02x4               g040(.a(new_n131), .b(new_n129), .c(new_n132), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n98), .c(new_n127), .d(new_n100), .o1(new_n137));
  xorc02aa1n02x5               g042(.a(\a[12] ), .b(\b[11] ), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n137), .c(new_n135), .out0(\s[12] ));
  xorc02aa1n02x5               g044(.a(\a[13] ), .b(\b[12] ), .out0(new_n140));
  and002aa1n02x5               g045(.a(\b[3] ), .b(\a[4] ), .o(new_n141));
  oaoi13aa1n02x5               g046(.a(new_n141), .b(new_n111), .c(new_n110), .d(new_n106), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n116), .b(new_n115), .out0(new_n143));
  xorc02aa1n02x5               g048(.a(\a[7] ), .b(\b[6] ), .out0(new_n144));
  nanp02aa1n02x5               g049(.a(new_n144), .b(new_n143), .o1(new_n145));
  nano22aa1n02x4               g050(.a(new_n145), .b(new_n114), .c(new_n119), .out0(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n122), .clkout(new_n147));
  norp02aa1n02x5               g052(.a(\b[5] ), .b(\a[6] ), .o1(new_n148));
  oab012aa1n02x4               g053(.a(new_n148), .b(\a[5] ), .c(\b[4] ), .out0(new_n149));
  nona23aa1n02x4               g054(.a(new_n144), .b(new_n143), .c(new_n149), .d(new_n147), .out0(new_n150));
  nona22aa1n02x4               g055(.a(new_n150), .b(new_n121), .c(new_n115), .out0(new_n151));
  oai022aa1n02x5               g056(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n152));
  nona23aa1n02x4               g057(.a(new_n136), .b(new_n138), .c(new_n152), .d(new_n99), .out0(new_n153));
  160nm_ficinv00aa1n08x5       g058(.clk(new_n153), .clkout(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n151), .c(new_n142), .d(new_n146), .o1(new_n155));
  norp02aa1n02x5               g060(.a(\b[11] ), .b(\a[12] ), .o1(new_n156));
  aoi112aa1n02x5               g061(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n157));
  aoi113aa1n02x5               g062(.a(new_n157), .b(new_n156), .c(new_n136), .d(new_n138), .e(new_n152), .o1(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n140), .b(new_n155), .c(new_n158), .out0(\s[13] ));
  orn002aa1n02x5               g064(.a(\a[13] ), .b(\b[12] ), .o(new_n160));
  oai012aa1n02x5               g065(.a(new_n125), .b(new_n113), .c(new_n120), .o1(new_n161));
  nanp03aa1n02x5               g066(.a(new_n136), .b(new_n138), .c(new_n152), .o1(new_n162));
  nona22aa1n02x4               g067(.a(new_n162), .b(new_n157), .c(new_n156), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n140), .b(new_n163), .c(new_n161), .d(new_n154), .o1(new_n164));
  xorc02aa1n02x5               g069(.a(\a[14] ), .b(\b[13] ), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n164), .c(new_n160), .out0(\s[14] ));
  and002aa1n02x5               g071(.a(new_n165), .b(new_n140), .o(new_n167));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n167), .clkout(new_n168));
  nanp02aa1n02x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  oai022aa1n02x5               g074(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(new_n170), .b(new_n169), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n168), .c(new_n155), .d(new_n158), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  norp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n178), .clkout(new_n179));
  aoi112aa1n02x5               g084(.a(new_n179), .b(new_n174), .c(new_n172), .d(new_n175), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n179), .b(new_n174), .c(new_n172), .d(new_n175), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n181), .b(new_n180), .out0(\s[16] ));
  nano23aa1n02x4               g087(.a(new_n174), .b(new_n176), .c(new_n177), .d(new_n175), .out0(new_n183));
  nanp03aa1n02x5               g088(.a(new_n183), .b(new_n140), .c(new_n165), .o1(new_n184));
  norp02aa1n02x5               g089(.a(new_n184), .b(new_n153), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n151), .c(new_n142), .d(new_n146), .o1(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(new_n184), .clkout(new_n187));
  nona23aa1n02x4               g092(.a(new_n177), .b(new_n175), .c(new_n174), .d(new_n176), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n174), .b(new_n177), .o1(new_n189));
  oai122aa1n02x7               g094(.a(new_n189), .b(new_n188), .c(new_n171), .d(\b[15] ), .e(\a[16] ), .o1(new_n190));
  aoi012aa1n02x5               g095(.a(new_n190), .b(new_n163), .c(new_n187), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(new_n186), .b(new_n191), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g098(.clk(\a[18] ), .clkout(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(\a[17] ), .clkout(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(\b[16] ), .clkout(new_n196));
  oaoi03aa1n02x5               g101(.a(new_n195), .b(new_n196), .c(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n194), .out0(\s[18] ));
  xroi22aa1d04x5               g103(.a(new_n195), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n199));
  nanp02aa1n02x5               g104(.a(new_n196), .b(new_n195), .o1(new_n200));
  oaoi03aa1n02x5               g105(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n201));
  norp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n201), .c(new_n192), .d(new_n199), .o1(new_n205));
  aoi112aa1n02x5               g110(.a(new_n204), .b(new_n201), .c(new_n192), .d(new_n199), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n205), .b(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanp02aa1n02x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  nona22aa1n02x4               g116(.a(new_n205), .b(new_n211), .c(new_n202), .out0(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n211), .clkout(new_n213));
  oaoi13aa1n02x5               g118(.a(new_n213), .b(new_n205), .c(\a[19] ), .d(\b[18] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n212), .b(new_n214), .out0(\s[20] ));
  nano23aa1n02x4               g120(.a(new_n202), .b(new_n209), .c(new_n210), .d(new_n203), .out0(new_n216));
  nanp02aa1n02x5               g121(.a(new_n199), .b(new_n216), .o1(new_n217));
  oai022aa1n02x5               g122(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n218));
  oaib12aa1n02x5               g123(.a(new_n218), .b(new_n194), .c(\b[17] ), .out0(new_n219));
  nona23aa1n02x4               g124(.a(new_n210), .b(new_n203), .c(new_n202), .d(new_n209), .out0(new_n220));
  aoi012aa1n02x5               g125(.a(new_n209), .b(new_n202), .c(new_n210), .o1(new_n221));
  oai012aa1n02x5               g126(.a(new_n221), .b(new_n220), .c(new_n219), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n222), .clkout(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n217), .c(new_n186), .d(new_n191), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  xorc02aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  aoi112aa1n02x5               g133(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g136(.clk(\a[21] ), .clkout(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(\a[22] ), .clkout(new_n233));
  xroi22aa1d04x5               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  nanp03aa1n02x5               g139(.a(new_n234), .b(new_n199), .c(new_n216), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(\b[21] ), .clkout(new_n236));
  oaoi03aa1n02x5               g141(.a(new_n233), .b(new_n236), .c(new_n226), .o1(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n222), .c(new_n234), .o1(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n235), .c(new_n186), .d(new_n191), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  xorc02aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n242), .b(new_n244), .c(new_n240), .d(new_n243), .o1(new_n245));
  aoai13aa1n02x5               g150(.a(new_n244), .b(new_n242), .c(new_n240), .d(new_n243), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(\s[24] ));
  and002aa1n02x5               g152(.a(new_n244), .b(new_n243), .o(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  nano32aa1n02x4               g154(.a(new_n249), .b(new_n234), .c(new_n199), .d(new_n216), .out0(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n221), .clkout(new_n251));
  aoai13aa1n02x5               g156(.a(new_n234), .b(new_n251), .c(new_n216), .d(new_n201), .o1(new_n252));
  orn002aa1n02x5               g157(.a(\a[23] ), .b(\b[22] ), .o(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n253), .carry(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n249), .c(new_n252), .d(new_n237), .o1(new_n255));
  xorc02aa1n02x5               g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n255), .c(new_n192), .d(new_n250), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(new_n256), .b(new_n255), .c(new_n192), .d(new_n250), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n257), .b(new_n258), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  nona22aa1n02x4               g166(.a(new_n257), .b(new_n261), .c(new_n260), .out0(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n260), .clkout(new_n263));
  aobi12aa1n02x5               g168(.a(new_n261), .b(new_n257), .c(new_n263), .out0(new_n264));
  norb02aa1n02x5               g169(.a(new_n262), .b(new_n264), .out0(\s[26] ));
  oabi12aa1n02x5               g170(.a(new_n190), .b(new_n158), .c(new_n184), .out0(new_n266));
  and002aa1n02x5               g171(.a(new_n261), .b(new_n256), .o(new_n267));
  nano22aa1n02x4               g172(.a(new_n235), .b(new_n248), .c(new_n267), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n266), .c(new_n161), .d(new_n185), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n263), .carry(new_n270));
  aobi12aa1n02x5               g175(.a(new_n270), .b(new_n255), .c(new_n267), .out0(new_n271));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n273), .b(new_n272), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n271), .c(new_n269), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n272), .clkout(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n268), .clkout(new_n278));
  aoi012aa1n02x5               g183(.a(new_n278), .b(new_n186), .c(new_n191), .o1(new_n279));
  aoai13aa1n02x5               g184(.a(new_n248), .b(new_n238), .c(new_n222), .d(new_n234), .o1(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n267), .clkout(new_n281));
  aoai13aa1n02x5               g186(.a(new_n270), .b(new_n281), .c(new_n280), .d(new_n254), .o1(new_n282));
  oai012aa1n02x5               g187(.a(new_n273), .b(new_n282), .c(new_n279), .o1(new_n283));
  aoi012aa1n02x5               g188(.a(new_n277), .b(new_n283), .c(new_n276), .o1(new_n284));
  aobi12aa1n02x5               g189(.a(new_n273), .b(new_n271), .c(new_n269), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n285), .b(new_n276), .c(new_n277), .out0(new_n286));
  norp02aa1n02x5               g191(.a(new_n284), .b(new_n286), .o1(\s[28] ));
  nano22aa1n02x4               g192(.a(new_n277), .b(new_n276), .c(new_n273), .out0(new_n288));
  oai012aa1n02x5               g193(.a(new_n288), .b(new_n282), .c(new_n279), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  aoi012aa1n02x5               g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x5               g197(.a(new_n288), .b(new_n271), .c(new_n269), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n274), .b(new_n291), .c(new_n277), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n297), .b(new_n282), .c(new_n279), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x5               g206(.a(new_n297), .b(new_n271), .c(new_n269), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n02x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  norb03aa1n02x5               g209(.a(new_n288), .b(new_n300), .c(new_n291), .out0(new_n305));
  aobi12aa1n02x5               g210(.a(new_n305), .b(new_n271), .c(new_n269), .out0(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n306), .b(new_n307), .c(new_n308), .out0(new_n309));
  oai012aa1n02x5               g214(.a(new_n305), .b(new_n282), .c(new_n279), .o1(new_n310));
  aoi012aa1n02x5               g215(.a(new_n308), .b(new_n310), .c(new_n307), .o1(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n309), .o1(\s[31] ));
  xnbna2aa1n03x5               g217(.a(new_n110), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n142), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g221(.a(\a[5] ), .b(\b[4] ), .c(new_n113), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_ficinv00aa1n08x5       g223(.clk(new_n114), .clkout(new_n319));
  oai112aa1n02x5               g224(.a(new_n122), .b(new_n144), .c(new_n317), .d(new_n319), .o1(new_n320));
  oaoi13aa1n02x5               g225(.a(new_n144), .b(new_n122), .c(new_n317), .d(new_n319), .o1(new_n321));
  norb02aa1n02x5               g226(.a(new_n320), .b(new_n321), .out0(\s[7] ));
  orn002aa1n02x5               g227(.a(\a[7] ), .b(\b[6] ), .o(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n143), .b(new_n320), .c(new_n323), .out0(\s[8] ));
  xorb03aa1n02x5               g229(.a(new_n161), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


