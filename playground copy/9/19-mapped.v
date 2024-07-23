// Benchmark "adder" written by ABC on Thu Jul 11 11:33:14 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n324;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nano23aa1n02x4               g006(.a(new_n98), .b(new_n100), .c(new_n101), .d(new_n99), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  nona22aa1n02x4               g009(.a(new_n102), .b(new_n103), .c(new_n104), .out0(new_n105));
  160nm_ficinv00aa1n08x5       g010(.clk(\a[2] ), .clkout(new_n106));
  160nm_ficinv00aa1n08x5       g011(.clk(\b[1] ), .clkout(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  oao003aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .carry(new_n109));
  norp02aa1n02x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nano23aa1n02x4               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  aoi012aa1n02x5               g019(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n115));
  aobi12aa1n02x5               g020(.a(new_n115), .b(new_n114), .c(new_n109), .out0(new_n116));
  nona23aa1n02x4               g021(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[6] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[5] ), .clkout(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n99), .b(new_n100), .c(new_n98), .o1(new_n122));
  oai012aa1n02x5               g027(.a(new_n122), .b(new_n117), .c(new_n121), .o1(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(new_n123), .clkout(new_n124));
  oai012aa1n02x5               g029(.a(new_n124), .b(new_n116), .c(new_n105), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n97), .b(new_n125), .c(new_n126), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  norp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nano23aa1n02x4               g038(.a(new_n97), .b(new_n132), .c(new_n133), .d(new_n126), .out0(new_n134));
  aoi012aa1n02x5               g039(.a(new_n132), .b(new_n97), .c(new_n133), .o1(new_n135));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n135), .clkout(new_n136));
  aoai13aa1n02x5               g041(.a(new_n131), .b(new_n136), .c(new_n125), .d(new_n134), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n131), .b(new_n136), .c(new_n125), .d(new_n134), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g044(.clk(new_n129), .clkout(new_n140));
  xorc02aa1n02x5               g045(.a(\a[12] ), .b(\b[11] ), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n137), .c(new_n140), .out0(\s[12] ));
  norp03aa1n02x5               g047(.a(new_n117), .b(new_n103), .c(new_n104), .o1(new_n143));
  oaoi03aa1n02x5               g048(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n144));
  nona23aa1n02x4               g049(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n145));
  oai012aa1n02x5               g050(.a(new_n115), .b(new_n145), .c(new_n144), .o1(new_n146));
  and003aa1n02x5               g051(.a(new_n134), .b(new_n131), .c(new_n141), .o(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n123), .c(new_n146), .d(new_n143), .o1(new_n148));
  oaoi03aa1n02x5               g053(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n149));
  aoi013aa1n02x4               g054(.a(new_n149), .b(new_n136), .c(new_n141), .d(new_n131), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n148), .b(new_n150), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nona23aa1n02x4               g063(.a(new_n158), .b(new_n154), .c(new_n153), .d(new_n157), .out0(new_n159));
  aoi012aa1n02x5               g064(.a(new_n157), .b(new_n153), .c(new_n158), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n148), .d(new_n150), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  160nm_ficinv00aa1n08x5       g070(.clk(new_n165), .clkout(new_n166));
  xorc02aa1n02x5               g071(.a(\a[16] ), .b(\b[15] ), .out0(new_n167));
  aoi112aa1n02x5               g072(.a(new_n167), .b(new_n163), .c(new_n161), .d(new_n166), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n167), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(\s[16] ));
  nano23aa1n02x4               g075(.a(new_n153), .b(new_n157), .c(new_n158), .d(new_n154), .out0(new_n171));
  nanp03aa1n02x5               g076(.a(new_n171), .b(new_n166), .c(new_n167), .o1(new_n172));
  nano32aa1n02x4               g077(.a(new_n172), .b(new_n141), .c(new_n134), .d(new_n131), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n123), .c(new_n143), .d(new_n146), .o1(new_n174));
  norp02aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(new_n175), .clkout(new_n176));
  xnrc02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .out0(new_n177));
  norp03aa1n02x5               g082(.a(new_n177), .b(new_n160), .c(new_n165), .o1(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(new_n178), .clkout(new_n179));
  aoi112aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(new_n180), .clkout(new_n181));
  aoi112aa1n02x5               g086(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n182));
  oai112aa1n02x5               g087(.a(new_n141), .b(new_n131), .c(new_n182), .d(new_n132), .o1(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(new_n149), .clkout(new_n184));
  aoi012aa1n02x5               g089(.a(new_n172), .b(new_n183), .c(new_n184), .o1(new_n185));
  nano32aa1n02x4               g090(.a(new_n185), .b(new_n179), .c(new_n181), .d(new_n176), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n174), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[18] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[17] ), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\b[16] ), .clkout(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d04x5               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  norp02aa1n02x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n196));
  norp02aa1n02x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n197), .clkout(new_n198));
  norp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoai13aa1n02x5               g106(.a(new_n201), .b(new_n198), .c(new_n187), .d(new_n194), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n201), .b(new_n198), .c(new_n187), .d(new_n194), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  nona22aa1n02x4               g113(.a(new_n202), .b(new_n208), .c(new_n199), .out0(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n199), .clkout(new_n210));
  aobi12aa1n02x5               g115(.a(new_n208), .b(new_n202), .c(new_n210), .out0(new_n211));
  norb02aa1n02x5               g116(.a(new_n209), .b(new_n211), .out0(\s[20] ));
  nona23aa1n02x4               g117(.a(new_n207), .b(new_n200), .c(new_n199), .d(new_n206), .out0(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  nanp02aa1n02x5               g119(.a(new_n194), .b(new_n214), .o1(new_n215));
  oai012aa1n02x5               g120(.a(new_n207), .b(new_n206), .c(new_n199), .o1(new_n216));
  oai012aa1n02x5               g121(.a(new_n216), .b(new_n213), .c(new_n197), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n215), .c(new_n174), .d(new_n186), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoi112aa1n02x5               g128(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n223), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(\s[22] ));
  nanp02aa1n02x5               g131(.a(new_n223), .b(new_n222), .o1(new_n227));
  nanb03aa1n02x5               g132(.a(new_n227), .b(new_n194), .c(new_n214), .out0(new_n228));
  oai112aa1n02x5               g133(.a(new_n201), .b(new_n208), .c(new_n196), .d(new_n195), .o1(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[22] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\b[21] ), .clkout(new_n231));
  oao003aa1n02x5               g136(.a(new_n230), .b(new_n231), .c(new_n221), .carry(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  aoai13aa1n02x5               g138(.a(new_n233), .b(new_n227), .c(new_n229), .d(new_n216), .o1(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n234), .clkout(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n228), .c(new_n174), .d(new_n186), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  xorc02aa1n02x5               g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  xorc02aa1n02x5               g144(.a(\a[24] ), .b(\b[23] ), .out0(new_n240));
  aoi112aa1n02x5               g145(.a(new_n238), .b(new_n240), .c(new_n236), .d(new_n239), .o1(new_n241));
  aoai13aa1n02x5               g146(.a(new_n240), .b(new_n238), .c(new_n236), .d(new_n239), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(\s[24] ));
  and002aa1n02x5               g148(.a(new_n240), .b(new_n239), .o(new_n244));
  nona23aa1n02x4               g149(.a(new_n244), .b(new_n194), .c(new_n227), .d(new_n213), .out0(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(\a[24] ), .clkout(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(\b[23] ), .clkout(new_n247));
  oao003aa1n02x5               g152(.a(new_n246), .b(new_n247), .c(new_n238), .carry(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n234), .c(new_n244), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n245), .c(new_n174), .d(new_n186), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n252), .b(new_n254), .c(new_n250), .d(new_n253), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n254), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(\s[26] ));
  xnrc02aa1n02x5               g162(.a(\b[11] ), .b(\a[12] ), .out0(new_n258));
  norb03aa1n02x5               g163(.a(new_n131), .b(new_n258), .c(new_n135), .out0(new_n259));
  norp03aa1n02x5               g164(.a(new_n159), .b(new_n177), .c(new_n165), .o1(new_n260));
  oai012aa1n02x5               g165(.a(new_n260), .b(new_n259), .c(new_n149), .o1(new_n261));
  nona32aa1n02x4               g166(.a(new_n261), .b(new_n180), .c(new_n178), .d(new_n175), .out0(new_n262));
  and002aa1n02x5               g167(.a(new_n254), .b(new_n253), .o(new_n263));
  nano22aa1n02x4               g168(.a(new_n228), .b(new_n244), .c(new_n263), .out0(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n262), .c(new_n125), .d(new_n173), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n263), .b(new_n248), .c(new_n234), .d(new_n244), .o1(new_n266));
  oai022aa1n02x5               g171(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n267));
  aob012aa1n02x5               g172(.a(new_n267), .b(\b[25] ), .c(\a[26] ), .out0(new_n268));
  xorc02aa1n02x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n269), .clkout(new_n270));
  aoi013aa1n02x4               g175(.a(new_n270), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n227), .clkout(new_n272));
  aoai13aa1n02x5               g177(.a(new_n244), .b(new_n232), .c(new_n217), .d(new_n272), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n248), .clkout(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n263), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n268), .b(new_n275), .c(new_n273), .d(new_n274), .o1(new_n276));
  aoi112aa1n02x5               g181(.a(new_n276), .b(new_n269), .c(new_n187), .d(new_n264), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n271), .b(new_n277), .o1(\s[27] ));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n279), .clkout(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n271), .b(new_n280), .c(new_n281), .out0(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n264), .clkout(new_n283));
  aoi012aa1n02x5               g188(.a(new_n283), .b(new_n174), .c(new_n186), .o1(new_n284));
  oai012aa1n02x5               g189(.a(new_n269), .b(new_n276), .c(new_n284), .o1(new_n285));
  aoi012aa1n02x5               g190(.a(new_n281), .b(new_n285), .c(new_n280), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n282), .o1(\s[28] ));
  norb02aa1n02x5               g192(.a(new_n269), .b(new_n281), .out0(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(new_n288), .clkout(new_n289));
  aoi013aa1n02x4               g194(.a(new_n289), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n280), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n290), .b(new_n291), .c(new_n292), .out0(new_n293));
  oai012aa1n02x5               g198(.a(new_n288), .b(new_n276), .c(new_n284), .o1(new_n294));
  aoi012aa1n02x5               g199(.a(new_n292), .b(new_n294), .c(new_n291), .o1(new_n295));
  norp02aa1n02x5               g200(.a(new_n295), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g202(.a(new_n269), .b(new_n292), .c(new_n281), .out0(new_n298));
  160nm_ficinv00aa1n08x5       g203(.clk(new_n298), .clkout(new_n299));
  aoi013aa1n02x4               g204(.a(new_n299), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  oai012aa1n02x5               g208(.a(new_n298), .b(new_n276), .c(new_n284), .o1(new_n304));
  aoi012aa1n02x5               g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n303), .o1(\s[30] ));
  norb02aa1n02x5               g211(.a(new_n298), .b(new_n302), .out0(new_n307));
  160nm_ficinv00aa1n08x5       g212(.clk(new_n307), .clkout(new_n308));
  aoi013aa1n02x4               g213(.a(new_n308), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  nano22aa1n02x4               g216(.a(new_n309), .b(new_n310), .c(new_n311), .out0(new_n312));
  oai012aa1n02x5               g217(.a(new_n307), .b(new_n276), .c(new_n284), .o1(new_n313));
  aoi012aa1n02x5               g218(.a(new_n311), .b(new_n313), .c(new_n310), .o1(new_n314));
  norp02aa1n02x5               g219(.a(new_n314), .b(new_n312), .o1(\s[31] ));
  xnrb03aa1n02x5               g220(.a(new_n144), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n144), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n146), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g224(.a(\a[5] ), .b(\b[4] ), .c(new_n116), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai013aa1n02x4               g226(.a(new_n121), .b(new_n116), .c(new_n103), .d(new_n104), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g228(.a(new_n100), .b(new_n322), .c(new_n101), .o1(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


