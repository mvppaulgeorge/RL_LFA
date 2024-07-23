// Benchmark "adder" written by ABC on Thu Jul 11 12:00:13 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n311, new_n312,
    new_n314, new_n316;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[2] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[1] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n02x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n02x5               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[6] ), .b(\b[5] ), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[5] ), .b(\b[4] ), .out0(new_n114));
  nano22aa1n02x4               g019(.a(new_n112), .b(new_n113), .c(new_n114), .out0(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  aoi112aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(new_n117), .b(new_n116), .o1(new_n118));
  oai012aa1n02x5               g023(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n119), .b(new_n112), .c(new_n118), .o1(new_n120));
  aoi012aa1n02x5               g025(.a(new_n120), .b(new_n115), .c(new_n107), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .c(new_n121), .o1(new_n122));
  xorb03aa1n02x5               g027(.a(new_n122), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nano23aa1n02x4               g032(.a(new_n124), .b(new_n126), .c(new_n127), .d(new_n125), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n120), .c(new_n115), .d(new_n107), .o1(new_n129));
  aoi012aa1n02x5               g034(.a(new_n124), .b(new_n126), .c(new_n125), .o1(new_n130));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n129), .c(new_n130), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n131), .clkout(new_n135));
  nanp02aa1n02x5               g040(.a(new_n115), .b(new_n107), .o1(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n120), .clkout(new_n137));
  nanp02aa1n02x5               g042(.a(new_n136), .b(new_n137), .o1(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n130), .clkout(new_n139));
  aoai13aa1n02x5               g044(.a(new_n133), .b(new_n139), .c(new_n138), .d(new_n128), .o1(new_n140));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n135), .out0(\s[12] ));
  nano23aa1n02x4               g049(.a(new_n131), .b(new_n141), .c(new_n142), .d(new_n132), .out0(new_n145));
  nona23aa1n02x4               g050(.a(new_n142), .b(new_n132), .c(new_n131), .d(new_n141), .out0(new_n146));
  oaoi03aa1n02x5               g051(.a(\a[12] ), .b(\b[11] ), .c(new_n135), .o1(new_n147));
  oabi12aa1n02x5               g052(.a(new_n147), .b(new_n146), .c(new_n130), .out0(new_n148));
  aoi013aa1n02x4               g053(.a(new_n148), .b(new_n138), .c(new_n128), .d(new_n145), .o1(new_n149));
  norp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n150), .clkout(new_n151));
  nanp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n149), .b(new_n152), .c(new_n151), .out0(\s[13] ));
  oaoi03aa1n02x5               g058(.a(\a[13] ), .b(\b[12] ), .c(new_n149), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1n02x4               g062(.a(new_n150), .b(new_n156), .c(new_n157), .d(new_n152), .out0(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n147), .c(new_n145), .d(new_n139), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n156), .b(new_n150), .c(new_n157), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(new_n159), .b(new_n160), .o1(new_n161));
  nano32aa1n02x4               g066(.a(new_n121), .b(new_n158), .c(new_n128), .d(new_n145), .out0(new_n162));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  nanp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  oai112aa1n02x5               g070(.a(new_n164), .b(new_n165), .c(new_n162), .d(new_n161), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n162), .b(new_n161), .c(new_n164), .d(new_n165), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[15] ));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n166), .c(new_n164), .out0(\s[16] ));
  nano23aa1n02x4               g077(.a(new_n163), .b(new_n169), .c(new_n170), .d(new_n165), .out0(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n173), .clkout(new_n174));
  nano32aa1n02x4               g079(.a(new_n174), .b(new_n158), .c(new_n145), .d(new_n128), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(new_n160), .clkout(new_n177));
  aoai13aa1n02x5               g082(.a(new_n173), .b(new_n177), .c(new_n148), .d(new_n158), .o1(new_n178));
  oai012aa1n02x5               g083(.a(new_n170), .b(new_n169), .c(new_n163), .o1(new_n179));
  nanp03aa1n02x5               g084(.a(new_n176), .b(new_n178), .c(new_n179), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  norp02aa1n02x5               g086(.a(\b[16] ), .b(\a[17] ), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(\b[16] ), .b(\a[17] ), .o1(new_n183));
  oai012aa1n02x5               g088(.a(new_n183), .b(new_n180), .c(new_n182), .o1(new_n184));
  xnrb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  aobi12aa1n02x5               g090(.a(new_n179), .b(new_n161), .c(new_n173), .out0(new_n186));
  norp02aa1n02x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  nano23aa1n02x4               g093(.a(new_n182), .b(new_n187), .c(new_n188), .d(new_n183), .out0(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n189), .clkout(new_n190));
  aoi012aa1n02x5               g095(.a(new_n187), .b(new_n182), .c(new_n188), .o1(new_n191));
  aoai13aa1n02x5               g096(.a(new_n191), .b(new_n190), .c(new_n186), .d(new_n176), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g098(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  norp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  aoi112aa1n02x5               g105(.a(new_n195), .b(new_n200), .c(new_n192), .d(new_n197), .o1(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n195), .clkout(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n191), .clkout(new_n203));
  aoai13aa1n02x5               g108(.a(new_n197), .b(new_n203), .c(new_n180), .d(new_n189), .o1(new_n204));
  aobi12aa1n02x5               g109(.a(new_n200), .b(new_n204), .c(new_n202), .out0(new_n205));
  norp02aa1n02x5               g110(.a(new_n205), .b(new_n201), .o1(\s[20] ));
  nano23aa1n02x4               g111(.a(new_n195), .b(new_n198), .c(new_n199), .d(new_n196), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n207), .b(new_n189), .o1(new_n208));
  nona23aa1n02x4               g113(.a(new_n199), .b(new_n196), .c(new_n195), .d(new_n198), .out0(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[20] ), .b(\b[19] ), .c(new_n202), .o1(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  oai012aa1n02x5               g116(.a(new_n211), .b(new_n209), .c(new_n191), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n208), .c(new_n186), .d(new_n176), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  aoi112aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n214), .d(new_n217), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n216), .clkout(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n208), .clkout(new_n221));
  aoai13aa1n02x5               g126(.a(new_n217), .b(new_n212), .c(new_n180), .d(new_n221), .o1(new_n222));
  aobi12aa1n02x5               g127(.a(new_n218), .b(new_n222), .c(new_n220), .out0(new_n223));
  norp02aa1n02x5               g128(.a(new_n223), .b(new_n219), .o1(\s[22] ));
  nano22aa1n02x4               g129(.a(new_n208), .b(new_n217), .c(new_n218), .out0(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\a[21] ), .clkout(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(\a[22] ), .clkout(new_n228));
  xroi22aa1d04x5               g133(.a(new_n227), .b(\b[20] ), .c(new_n228), .d(\b[21] ), .out0(new_n229));
  oao003aa1n02x5               g134(.a(\a[22] ), .b(\b[21] ), .c(new_n220), .carry(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n230), .clkout(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n212), .c(new_n229), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n226), .c(new_n186), .d(new_n176), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  xorc02aa1n02x5               g141(.a(\a[24] ), .b(\b[23] ), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n235), .clkout(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n232), .clkout(new_n240));
  aoai13aa1n02x5               g145(.a(new_n236), .b(new_n240), .c(new_n180), .d(new_n225), .o1(new_n241));
  aobi12aa1n02x5               g146(.a(new_n237), .b(new_n241), .c(new_n239), .out0(new_n242));
  norp02aa1n02x5               g147(.a(new_n242), .b(new_n238), .o1(\s[24] ));
  and002aa1n02x5               g148(.a(new_n237), .b(new_n236), .o(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  nano32aa1n02x4               g150(.a(new_n245), .b(new_n229), .c(new_n207), .d(new_n189), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  aoai13aa1n02x5               g152(.a(new_n229), .b(new_n210), .c(new_n207), .d(new_n203), .o1(new_n248));
  oao003aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .c(new_n239), .carry(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n245), .c(new_n248), .d(new_n230), .o1(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n250), .clkout(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n247), .c(new_n186), .d(new_n176), .o1(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  xorc02aa1n02x5               g160(.a(\a[26] ), .b(\b[25] ), .out0(new_n256));
  aoi112aa1n02x5               g161(.a(new_n254), .b(new_n256), .c(new_n252), .d(new_n255), .o1(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n254), .clkout(new_n258));
  aoai13aa1n02x5               g163(.a(new_n255), .b(new_n250), .c(new_n180), .d(new_n246), .o1(new_n259));
  aobi12aa1n02x5               g164(.a(new_n256), .b(new_n259), .c(new_n258), .out0(new_n260));
  norp02aa1n02x5               g165(.a(new_n260), .b(new_n257), .o1(\s[26] ));
  aoai13aa1n02x5               g166(.a(new_n179), .b(new_n174), .c(new_n159), .d(new_n160), .o1(new_n262));
  and002aa1n02x5               g167(.a(new_n256), .b(new_n255), .o(new_n263));
  nano23aa1n02x4               g168(.a(new_n208), .b(new_n245), .c(new_n263), .d(new_n229), .out0(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n262), .c(new_n138), .d(new_n175), .o1(new_n265));
  oao003aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n258), .carry(new_n266));
  aobi12aa1n02x5               g171(.a(new_n266), .b(new_n250), .c(new_n263), .out0(new_n267));
  norp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  nanp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  norb02aa1n02x5               g174(.a(new_n269), .b(new_n268), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n265), .c(new_n267), .out0(\s[27] ));
  xorc02aa1n02x5               g176(.a(\a[28] ), .b(\b[27] ), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n244), .b(new_n231), .c(new_n212), .d(new_n229), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n263), .clkout(new_n274));
  aoai13aa1n02x5               g179(.a(new_n266), .b(new_n274), .c(new_n273), .d(new_n249), .o1(new_n275));
  aoi112aa1n02x5               g180(.a(new_n275), .b(new_n268), .c(new_n180), .d(new_n264), .o1(new_n276));
  nano22aa1n02x4               g181(.a(new_n276), .b(new_n269), .c(new_n272), .out0(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n268), .clkout(new_n278));
  nanp03aa1n02x5               g183(.a(new_n265), .b(new_n267), .c(new_n278), .o1(new_n279));
  aoi012aa1n02x5               g184(.a(new_n272), .b(new_n279), .c(new_n269), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n277), .o1(\s[28] ));
  and002aa1n02x5               g186(.a(new_n272), .b(new_n270), .o(new_n282));
  aoai13aa1n02x5               g187(.a(new_n282), .b(new_n275), .c(new_n180), .d(new_n264), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n283), .c(new_n284), .o1(new_n286));
  aobi12aa1n02x5               g191(.a(new_n282), .b(new_n265), .c(new_n267), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n284), .c(new_n285), .out0(new_n288));
  norp02aa1n02x5               g193(.a(new_n286), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g195(.a(new_n285), .b(new_n272), .c(new_n270), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n275), .c(new_n180), .d(new_n264), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n02x5               g200(.a(new_n291), .b(new_n265), .c(new_n267), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n295), .b(new_n297), .o1(\s[30] ));
  nano23aa1n02x4               g203(.a(new_n294), .b(new_n285), .c(new_n272), .d(new_n270), .out0(new_n299));
  aobi12aa1n02x5               g204(.a(new_n299), .b(new_n265), .c(new_n267), .out0(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n299), .b(new_n275), .c(new_n180), .d(new_n264), .o1(new_n304));
  aoi012aa1n02x5               g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  xnrb03aa1n02x5               g211(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g212(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g214(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai112aa1n02x5               g215(.a(new_n106), .b(new_n114), .c(new_n105), .d(new_n100), .o1(new_n311));
  aob012aa1n02x5               g216(.a(new_n311), .b(\b[4] ), .c(\a[5] ), .out0(new_n312));
  xnrc02aa1n02x5               g217(.a(new_n312), .b(new_n113), .out0(\s[6] ));
  oao003aa1n02x5               g218(.a(\a[6] ), .b(\b[5] ), .c(new_n312), .carry(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g220(.a(\a[7] ), .b(\b[6] ), .c(new_n314), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g222(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


