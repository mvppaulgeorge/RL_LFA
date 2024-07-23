// Benchmark "adder" written by ABC on Wed Jul 10 16:42:13 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n311, new_n313,
    new_n315;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  and002aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  orn002aa1n02x5               g005(.a(\a[3] ), .b(\b[2] ), .o(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(new_n101), .b(new_n102), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  oa0022aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n108));
  oaoi13aa1n02x5               g013(.a(new_n100), .b(new_n108), .c(new_n103), .d(new_n107), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  norp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nanb02aa1n02x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  norp03aa1n02x5               g023(.a(new_n114), .b(new_n115), .c(new_n118), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n109), .b(new_n119), .o1(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\a[6] ), .clkout(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\b[5] ), .clkout(new_n122));
  oaoi03aa1n02x5               g027(.a(new_n121), .b(new_n122), .c(new_n116), .o1(new_n123));
  160nm_fiao0012aa1n02p5x5     g028(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n124));
  oabi12aa1n02x5               g029(.a(new_n124), .b(new_n114), .c(new_n123), .out0(new_n125));
  160nm_ficinv00aa1n08x5       g030(.clk(new_n125), .clkout(new_n126));
  nanp02aa1n02x5               g031(.a(new_n120), .b(new_n126), .o1(new_n127));
  xorc02aa1n02x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n98), .b(new_n99), .c(new_n127), .d(new_n128), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n128), .b(new_n125), .c(new_n109), .d(new_n119), .o1(new_n130));
  nona22aa1n02x4               g035(.a(new_n130), .b(new_n99), .c(new_n98), .out0(new_n131));
  nanp02aa1n02x5               g036(.a(new_n129), .b(new_n131), .o1(\s[10] ));
  160nm_ficinv00aa1n08x5       g037(.clk(\a[10] ), .clkout(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(\b[9] ), .clkout(new_n134));
  norp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  oai112aa1n02x5               g042(.a(new_n131), .b(new_n137), .c(new_n134), .d(new_n133), .o1(new_n138));
  oaoi13aa1n02x5               g043(.a(new_n137), .b(new_n131), .c(new_n133), .d(new_n134), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n135), .clkout(new_n141));
  norp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  xnrc02aa1n02x5               g050(.a(\b[12] ), .b(\a[13] ), .out0(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n146), .clkout(new_n147));
  nano23aa1n02x4               g052(.a(new_n135), .b(new_n142), .c(new_n143), .d(new_n136), .out0(new_n148));
  nanp03aa1n02x5               g053(.a(new_n148), .b(new_n97), .c(new_n128), .o1(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n125), .c(new_n109), .d(new_n119), .o1(new_n151));
  nona23aa1n02x4               g056(.a(new_n143), .b(new_n136), .c(new_n135), .d(new_n142), .out0(new_n152));
  oaoi03aa1n02x5               g057(.a(new_n133), .b(new_n134), .c(new_n99), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n142), .b(new_n135), .c(new_n143), .o1(new_n154));
  oai012aa1n02x5               g059(.a(new_n154), .b(new_n152), .c(new_n153), .o1(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n147), .b(new_n151), .c(new_n156), .out0(\s[13] ));
  orn002aa1n02x5               g062(.a(\a[13] ), .b(\b[12] ), .o(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n146), .c(new_n151), .d(new_n156), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .out0(new_n161));
  norp02aa1n02x5               g066(.a(new_n161), .b(new_n146), .o1(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  oao003aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .carry(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n151), .d(new_n156), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  nano23aa1n02x4               g079(.a(new_n167), .b(new_n169), .c(new_n170), .d(new_n168), .out0(new_n175));
  nona22aa1n02x4               g080(.a(new_n175), .b(new_n161), .c(new_n146), .out0(new_n176));
  norp02aa1n02x5               g081(.a(new_n176), .b(new_n149), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n125), .c(new_n109), .d(new_n119), .o1(new_n178));
  oai012aa1n02x5               g083(.a(new_n170), .b(new_n169), .c(new_n167), .o1(new_n179));
  oaib12aa1n02x5               g084(.a(new_n179), .b(new_n164), .c(new_n175), .out0(new_n180));
  aoi013aa1n02x4               g085(.a(new_n180), .b(new_n155), .c(new_n162), .d(new_n175), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(new_n178), .b(new_n181), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[18] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[17] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\b[16] ), .clkout(new_n186));
  oaoi03aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  xroi22aa1d04x5               g093(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n189));
  nanp02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nona22aa1n02x4               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(new_n191));
  oaib12aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n184), .out0(new_n192));
  norp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nona22aa1n02x4               g107(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n203));
  orn002aa1n02x5               g108(.a(\a[19] ), .b(\b[18] ), .o(new_n204));
  aobi12aa1n02x5               g109(.a(new_n202), .b(new_n196), .c(new_n204), .out0(new_n205));
  norb02aa1n02x5               g110(.a(new_n203), .b(new_n205), .out0(\s[20] ));
  nano23aa1n02x4               g111(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n189), .b(new_n207), .o1(new_n208));
  norp02aa1n02x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  aoi013aa1n02x4               g114(.a(new_n209), .b(new_n190), .c(new_n185), .d(new_n186), .o1(new_n210));
  nona23aa1n02x4               g115(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n211));
  oaoi03aa1n02x5               g116(.a(\a[20] ), .b(\b[19] ), .c(new_n204), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  oai012aa1n02x5               g118(.a(new_n213), .b(new_n211), .c(new_n210), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n208), .c(new_n178), .d(new_n181), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[21] ), .clkout(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\a[22] ), .clkout(new_n225));
  xroi22aa1d04x5               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  nanp03aa1n02x5               g131(.a(new_n226), .b(new_n189), .c(new_n207), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\b[21] ), .clkout(new_n228));
  oao003aa1n02x5               g133(.a(new_n225), .b(new_n228), .c(new_n218), .carry(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n214), .c(new_n226), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n227), .c(new_n178), .d(new_n181), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g143(.clk(\a[23] ), .clkout(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(\a[24] ), .clkout(new_n240));
  xroi22aa1d04x5               g145(.a(new_n239), .b(\b[22] ), .c(new_n240), .d(\b[23] ), .out0(new_n241));
  nano22aa1n02x4               g146(.a(new_n208), .b(new_n226), .c(new_n241), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n226), .b(new_n212), .c(new_n207), .d(new_n192), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n229), .clkout(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n241), .clkout(new_n245));
  oai022aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n246));
  oaib12aa1n02x5               g151(.a(new_n246), .b(new_n240), .c(\b[23] ), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n245), .c(new_n243), .d(new_n244), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n248), .c(new_n182), .d(new_n242), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n249), .b(new_n248), .c(new_n182), .d(new_n242), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n250), .b(new_n251), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  nona22aa1n02x4               g159(.a(new_n250), .b(new_n254), .c(new_n253), .out0(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n253), .clkout(new_n256));
  aobi12aa1n02x5               g161(.a(new_n254), .b(new_n250), .c(new_n256), .out0(new_n257));
  norb02aa1n02x5               g162(.a(new_n255), .b(new_n257), .out0(\s[26] ));
  oabi12aa1n02x5               g163(.a(new_n180), .b(new_n156), .c(new_n176), .out0(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(\a[25] ), .clkout(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(\a[26] ), .clkout(new_n261));
  xroi22aa1d04x5               g166(.a(new_n260), .b(\b[24] ), .c(new_n261), .d(\b[25] ), .out0(new_n262));
  nano32aa1n02x4               g167(.a(new_n208), .b(new_n262), .c(new_n226), .d(new_n241), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n259), .c(new_n127), .d(new_n177), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n265));
  aobi12aa1n02x5               g170(.a(new_n265), .b(new_n248), .c(new_n262), .out0(new_n266));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  nanp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  norb02aa1n02x5               g173(.a(new_n268), .b(new_n267), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n266), .c(new_n264), .out0(\s[27] ));
  xorc02aa1n02x5               g175(.a(\a[28] ), .b(\b[27] ), .out0(new_n271));
  aobi12aa1n02x5               g176(.a(new_n263), .b(new_n178), .c(new_n181), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n241), .b(new_n229), .c(new_n214), .d(new_n226), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n262), .clkout(new_n274));
  aoai13aa1n02x5               g179(.a(new_n265), .b(new_n274), .c(new_n273), .d(new_n247), .o1(new_n275));
  norp03aa1n02x5               g180(.a(new_n275), .b(new_n272), .c(new_n267), .o1(new_n276));
  nano22aa1n02x4               g181(.a(new_n276), .b(new_n268), .c(new_n271), .out0(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n267), .clkout(new_n278));
  nanp03aa1n02x5               g183(.a(new_n266), .b(new_n264), .c(new_n278), .o1(new_n279));
  aoi012aa1n02x5               g184(.a(new_n271), .b(new_n279), .c(new_n268), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n277), .o1(\s[28] ));
  and002aa1n02x5               g186(.a(new_n271), .b(new_n269), .o(new_n282));
  oai012aa1n02x5               g187(.a(new_n282), .b(new_n275), .c(new_n272), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n283), .c(new_n284), .o1(new_n286));
  aobi12aa1n02x5               g191(.a(new_n282), .b(new_n266), .c(new_n264), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n284), .c(new_n285), .out0(new_n288));
  norp02aa1n02x5               g193(.a(new_n286), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g195(.a(new_n285), .b(new_n271), .c(new_n269), .out0(new_n291));
  oai012aa1n02x5               g196(.a(new_n291), .b(new_n275), .c(new_n272), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n02x5               g200(.a(new_n291), .b(new_n266), .c(new_n264), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n295), .b(new_n297), .o1(\s[30] ));
  nano23aa1n02x4               g203(.a(new_n294), .b(new_n285), .c(new_n271), .d(new_n269), .out0(new_n299));
  aobi12aa1n02x5               g204(.a(new_n299), .b(new_n266), .c(new_n264), .out0(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  oai012aa1n02x5               g208(.a(new_n299), .b(new_n275), .c(new_n272), .o1(new_n304));
  aoi012aa1n02x5               g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  xnbna2aa1n03x5               g211(.a(new_n107), .b(new_n101), .c(new_n102), .out0(\s[3] ));
  oaoi03aa1n02x5               g212(.a(\a[3] ), .b(\b[2] ), .c(new_n107), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g214(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g215(.a(new_n116), .b(new_n109), .c(new_n117), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[5] ), .c(new_n121), .out0(\s[6] ));
  oaoi03aa1n02x5               g217(.a(\a[6] ), .b(\b[5] ), .c(new_n311), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g219(.a(new_n112), .b(new_n313), .c(new_n113), .o1(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g221(.a(new_n128), .b(new_n120), .c(new_n126), .out0(\s[9] ));
endmodule


