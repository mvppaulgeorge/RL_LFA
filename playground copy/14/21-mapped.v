// Benchmark "adder" written by ABC on Thu Jul 11 11:55:54 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n322, new_n324, new_n325, new_n327;
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
  160nm_ficinv00aa1n08x5       g027(.clk(new_n113), .clkout(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  oabi12aa1n02x5               g029(.a(new_n124), .b(new_n115), .c(new_n122), .out0(new_n125));
  aoi112aa1n02x5               g030(.a(new_n125), .b(new_n99), .c(new_n110), .d(new_n118), .o1(new_n126));
  nano22aa1n02x4               g031(.a(new_n126), .b(new_n97), .c(new_n98), .out0(new_n127));
  oao003aa1n02x5               g032(.a(new_n100), .b(new_n101), .c(new_n102), .carry(new_n128));
  nano23aa1n02x4               g033(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n129), .b(new_n128), .o1(new_n130));
  nano23aa1n02x4               g035(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n131));
  nona22aa1n02x4               g036(.a(new_n131), .b(new_n116), .c(new_n117), .out0(new_n132));
  oao003aa1n02x5               g037(.a(new_n119), .b(new_n120), .c(new_n121), .carry(new_n133));
  aoi012aa1n02x5               g038(.a(new_n124), .b(new_n131), .c(new_n133), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n132), .c(new_n130), .d(new_n109), .o1(new_n135));
  oaoi13aa1n02x5               g040(.a(new_n97), .b(new_n98), .c(new_n135), .d(new_n99), .o1(new_n136));
  norp02aa1n02x5               g041(.a(new_n136), .b(new_n127), .o1(\s[10] ));
  norp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n138), .clkout(new_n139));
  nanp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  160nm_ficinv00aa1n08x5       g045(.clk(\a[10] ), .clkout(new_n141));
  160nm_ficinv00aa1n08x5       g046(.clk(\b[9] ), .clkout(new_n142));
  oao003aa1n02x5               g047(.a(new_n141), .b(new_n142), .c(new_n99), .carry(new_n143));
  norp02aa1n02x5               g048(.a(new_n127), .b(new_n143), .o1(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g050(.clk(\a[12] ), .clkout(new_n146));
  oaoi13aa1n02x5               g051(.a(new_n138), .b(new_n140), .c(new_n127), .d(new_n143), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[11] ), .c(new_n146), .out0(\s[12] ));
  nanp02aa1n02x5               g053(.a(new_n110), .b(new_n118), .o1(new_n149));
  norp02aa1n02x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nano23aa1n02x4               g056(.a(new_n138), .b(new_n150), .c(new_n151), .d(new_n140), .out0(new_n152));
  norb02aa1n02x5               g057(.a(new_n98), .b(new_n99), .out0(new_n153));
  nanp03aa1n02x5               g058(.a(new_n152), .b(new_n97), .c(new_n153), .o1(new_n154));
  oai012aa1n02x5               g059(.a(new_n151), .b(new_n150), .c(new_n138), .o1(new_n155));
  aobi12aa1n02x5               g060(.a(new_n155), .b(new_n152), .c(new_n143), .out0(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n154), .c(new_n149), .d(new_n134), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g063(.clk(\a[14] ), .clkout(new_n159));
  norp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  xorc02aa1n02x5               g065(.a(\a[13] ), .b(\b[12] ), .out0(new_n161));
  aoi012aa1n02x5               g066(.a(new_n160), .b(new_n157), .c(new_n161), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(new_n159), .out0(\s[14] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  xorc02aa1n02x5               g071(.a(\a[14] ), .b(\b[13] ), .out0(new_n167));
  and002aa1n02x5               g072(.a(new_n167), .b(new_n161), .o(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(\b[13] ), .clkout(new_n169));
  oaoi03aa1n02x5               g074(.a(new_n159), .b(new_n169), .c(new_n160), .o1(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(new_n170), .clkout(new_n171));
  aoai13aa1n02x5               g076(.a(new_n166), .b(new_n171), .c(new_n157), .d(new_n168), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n166), .b(new_n171), .c(new_n157), .d(new_n168), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  norp02aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  nona22aa1n02x4               g082(.a(new_n172), .b(new_n177), .c(new_n164), .out0(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n177), .clkout(new_n179));
  oaoi13aa1n02x5               g084(.a(new_n179), .b(new_n172), .c(\a[15] ), .d(\b[14] ), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n178), .b(new_n180), .out0(\s[16] ));
  nano23aa1n02x4               g086(.a(new_n164), .b(new_n175), .c(new_n176), .d(new_n165), .out0(new_n182));
  nanp03aa1n02x5               g087(.a(new_n182), .b(new_n161), .c(new_n167), .o1(new_n183));
  norp02aa1n02x5               g088(.a(new_n183), .b(new_n154), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n141), .b(new_n142), .c(new_n99), .o1(new_n186));
  nona23aa1n02x4               g091(.a(new_n151), .b(new_n140), .c(new_n138), .d(new_n150), .out0(new_n187));
  oai012aa1n02x5               g092(.a(new_n155), .b(new_n187), .c(new_n186), .o1(new_n188));
  oai012aa1n02x5               g093(.a(new_n176), .b(new_n175), .c(new_n164), .o1(new_n189));
  oaib12aa1n02x5               g094(.a(new_n189), .b(new_n170), .c(new_n182), .out0(new_n190));
  aoib12aa1n02x5               g095(.a(new_n190), .b(new_n188), .c(new_n183), .out0(new_n191));
  nanp02aa1n02x5               g096(.a(new_n185), .b(new_n191), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g098(.clk(\a[18] ), .clkout(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(\a[17] ), .clkout(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(\b[16] ), .clkout(new_n196));
  oaoi03aa1n02x5               g101(.a(new_n195), .b(new_n196), .c(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n194), .out0(\s[18] ));
  xroi22aa1d04x5               g103(.a(new_n195), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n199));
  norp02aa1n02x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  aoi112aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n201));
  norp02aa1n02x5               g106(.a(new_n201), .b(new_n200), .o1(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n202), .clkout(new_n203));
  norp02aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  aoai13aa1n02x5               g111(.a(new_n206), .b(new_n203), .c(new_n192), .d(new_n199), .o1(new_n207));
  aoi112aa1n02x5               g112(.a(new_n206), .b(new_n203), .c(new_n192), .d(new_n199), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nanp02aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  nona22aa1n02x4               g118(.a(new_n207), .b(new_n213), .c(new_n204), .out0(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n204), .clkout(new_n215));
  aobi12aa1n02x5               g120(.a(new_n213), .b(new_n207), .c(new_n215), .out0(new_n216));
  norb02aa1n02x5               g121(.a(new_n214), .b(new_n216), .out0(\s[20] ));
  nona23aa1n02x4               g122(.a(new_n212), .b(new_n205), .c(new_n204), .d(new_n211), .out0(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  nanp02aa1n02x5               g124(.a(new_n199), .b(new_n219), .o1(new_n220));
  oai012aa1n02x5               g125(.a(new_n212), .b(new_n211), .c(new_n204), .o1(new_n221));
  oai012aa1n02x5               g126(.a(new_n221), .b(new_n218), .c(new_n202), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n222), .clkout(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n220), .c(new_n185), .d(new_n191), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  xorc02aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  aoi112aa1n02x5               g133(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(\s[22] ));
  nanp02aa1n02x5               g136(.a(new_n228), .b(new_n227), .o1(new_n232));
  nanb03aa1n02x5               g137(.a(new_n232), .b(new_n199), .c(new_n219), .out0(new_n233));
  oai112aa1n02x5               g138(.a(new_n206), .b(new_n213), .c(new_n201), .d(new_n200), .o1(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(\a[22] ), .clkout(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(\b[21] ), .clkout(new_n236));
  oao003aa1n02x5               g141(.a(new_n235), .b(new_n236), .c(new_n226), .carry(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n232), .c(new_n234), .d(new_n221), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n233), .c(new_n185), .d(new_n191), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  xorc02aa1n02x5               g148(.a(\a[23] ), .b(\b[22] ), .out0(new_n244));
  xorc02aa1n02x5               g149(.a(\a[24] ), .b(\b[23] ), .out0(new_n245));
  aoi112aa1n02x5               g150(.a(new_n243), .b(new_n245), .c(new_n241), .d(new_n244), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n245), .b(new_n243), .c(new_n241), .d(new_n244), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(\s[24] ));
  and002aa1n02x5               g153(.a(new_n245), .b(new_n244), .o(new_n249));
  nona23aa1n02x4               g154(.a(new_n249), .b(new_n199), .c(new_n232), .d(new_n218), .out0(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(\a[24] ), .clkout(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(\b[23] ), .clkout(new_n252));
  oao003aa1n02x5               g157(.a(new_n251), .b(new_n252), .c(new_n243), .carry(new_n253));
  aoi012aa1n02x5               g158(.a(new_n253), .b(new_n239), .c(new_n249), .o1(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n250), .c(new_n185), .d(new_n191), .o1(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  xorc02aa1n02x5               g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  xorc02aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  aoi112aa1n02x5               g164(.a(new_n257), .b(new_n259), .c(new_n255), .d(new_n258), .o1(new_n260));
  aoai13aa1n02x5               g165(.a(new_n259), .b(new_n257), .c(new_n255), .d(new_n258), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n261), .b(new_n260), .out0(\s[26] ));
  aobi12aa1n02x5               g167(.a(new_n189), .b(new_n171), .c(new_n182), .out0(new_n263));
  oai012aa1n02x5               g168(.a(new_n263), .b(new_n156), .c(new_n183), .o1(new_n264));
  and002aa1n02x5               g169(.a(new_n259), .b(new_n258), .o(new_n265));
  nano22aa1n02x4               g170(.a(new_n233), .b(new_n249), .c(new_n265), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n264), .c(new_n135), .d(new_n184), .o1(new_n267));
  aoai13aa1n02x5               g172(.a(new_n265), .b(new_n253), .c(new_n239), .d(new_n249), .o1(new_n268));
  oai022aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n269));
  aob012aa1n02x5               g174(.a(new_n269), .b(\b[25] ), .c(\a[26] ), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n271), .clkout(new_n272));
  aoi013aa1n02x4               g177(.a(new_n272), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n266), .clkout(new_n274));
  aoi012aa1n02x5               g179(.a(new_n274), .b(new_n185), .c(new_n191), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n232), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n249), .b(new_n237), .c(new_n222), .d(new_n276), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n253), .clkout(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n265), .clkout(new_n279));
  aoai13aa1n02x5               g184(.a(new_n270), .b(new_n279), .c(new_n277), .d(new_n278), .o1(new_n280));
  norp03aa1n02x5               g185(.a(new_n280), .b(new_n275), .c(new_n271), .o1(new_n281));
  norp02aa1n02x5               g186(.a(new_n273), .b(new_n281), .o1(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n283), .clkout(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n273), .b(new_n284), .c(new_n285), .out0(new_n286));
  oai012aa1n02x5               g191(.a(new_n271), .b(new_n280), .c(new_n275), .o1(new_n287));
  aoi012aa1n02x5               g192(.a(new_n285), .b(new_n287), .c(new_n284), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n286), .o1(\s[28] ));
  norb02aa1n02x5               g194(.a(new_n271), .b(new_n285), .out0(new_n290));
  oai012aa1n02x5               g195(.a(new_n290), .b(new_n280), .c(new_n275), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n290), .clkout(new_n295));
  aoi013aa1n02x4               g200(.a(new_n295), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n292), .c(new_n293), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n294), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g204(.a(new_n271), .b(new_n293), .c(new_n285), .out0(new_n300));
  oai012aa1n02x5               g205(.a(new_n300), .b(new_n280), .c(new_n275), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  aoi012aa1n02x5               g208(.a(new_n303), .b(new_n301), .c(new_n302), .o1(new_n304));
  160nm_ficinv00aa1n08x5       g209(.clk(new_n300), .clkout(new_n305));
  aoi013aa1n02x4               g210(.a(new_n305), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n306));
  nano22aa1n02x4               g211(.a(new_n306), .b(new_n302), .c(new_n303), .out0(new_n307));
  norp02aa1n02x5               g212(.a(new_n304), .b(new_n307), .o1(\s[30] ));
  norb02aa1n02x5               g213(.a(new_n300), .b(new_n303), .out0(new_n309));
  160nm_ficinv00aa1n08x5       g214(.clk(new_n309), .clkout(new_n310));
  aoi013aa1n02x4               g215(.a(new_n310), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  nano22aa1n02x4               g218(.a(new_n311), .b(new_n312), .c(new_n313), .out0(new_n314));
  oai012aa1n02x5               g219(.a(new_n309), .b(new_n280), .c(new_n275), .o1(new_n315));
  aoi012aa1n02x5               g220(.a(new_n313), .b(new_n315), .c(new_n312), .o1(new_n316));
  norp02aa1n02x5               g221(.a(new_n316), .b(new_n314), .o1(\s[31] ));
  xnrb03aa1n02x5               g222(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g223(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g225(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoib12aa1n02x5               g226(.a(new_n121), .b(new_n110), .c(new_n117), .out0(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  norp02aa1n02x5               g228(.a(new_n117), .b(new_n116), .o1(new_n324));
  aoi012aa1n02x5               g229(.a(new_n133), .b(new_n110), .c(new_n324), .o1(new_n325));
  xnbna2aa1n03x5               g230(.a(new_n325), .b(new_n123), .c(new_n114), .out0(\s[7] ));
  oaoi03aa1n02x5               g231(.a(\a[7] ), .b(\b[6] ), .c(new_n325), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g233(.a(new_n135), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


