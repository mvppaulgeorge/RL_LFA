// Benchmark "adder" written by ABC on Thu Jul 11 11:14:21 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n308, new_n309, new_n311, new_n313;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nanp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\a[3] ), .clkout(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\b[2] ), .clkout(new_n103));
  nanp02aa1n02x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  oai012aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  160nm_ficinv00aa1n08x5       g015(.clk(\b[3] ), .clkout(new_n111));
  aboi22aa1n03x5               g016(.a(\a[4] ), .b(new_n111), .c(new_n102), .d(new_n103), .out0(new_n112));
  oai012aa1n02x5               g017(.a(new_n112), .b(new_n110), .c(new_n106), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .out0(new_n120));
  norp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  nona23aa1n02x4               g026(.a(new_n113), .b(new_n121), .c(new_n118), .d(new_n101), .out0(new_n122));
  norp02aa1n02x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n124));
  norp02aa1n02x5               g029(.a(new_n124), .b(new_n123), .o1(new_n125));
  160nm_fiao0012aa1n02p5x5     g030(.a(new_n114), .b(new_n116), .c(new_n115), .o(new_n126));
  oabi12aa1n02x5               g031(.a(new_n126), .b(new_n118), .c(new_n125), .out0(new_n127));
  xorc02aa1n02x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  160nm_ficinv00aa1n08x5       g033(.clk(new_n128), .clkout(new_n129));
  nona22aa1n02x4               g034(.a(new_n122), .b(new_n127), .c(new_n129), .out0(new_n130));
  xobna2aa1n03x5               g035(.a(new_n99), .b(new_n130), .c(new_n100), .out0(\s[10] ));
  aoai13aa1n02x5               g036(.a(new_n98), .b(new_n97), .c(new_n130), .d(new_n100), .o1(new_n132));
  xnrb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g038(.a(\a[11] ), .b(\b[10] ), .c(new_n132), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n127), .clkout(new_n136));
  norp02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nano23aa1n02x4               g045(.a(new_n137), .b(new_n139), .c(new_n140), .d(new_n138), .out0(new_n141));
  nanp03aa1n02x5               g046(.a(new_n141), .b(new_n99), .c(new_n128), .o1(new_n142));
  orn002aa1n02x5               g047(.a(\a[9] ), .b(\b[8] ), .o(new_n143));
  oaoi03aa1n02x5               g048(.a(\a[10] ), .b(\b[9] ), .c(new_n143), .o1(new_n144));
  160nm_fiao0012aa1n02p5x5     g049(.a(new_n139), .b(new_n137), .c(new_n140), .o(new_n145));
  aoi012aa1n02x5               g050(.a(new_n145), .b(new_n141), .c(new_n144), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n142), .c(new_n122), .d(new_n136), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g053(.clk(\a[14] ), .clkout(new_n149));
  norp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  xnrc02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .out0(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(new_n151), .clkout(new_n152));
  aoi012aa1n02x5               g057(.a(new_n150), .b(new_n147), .c(new_n152), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(new_n149), .out0(\s[14] ));
  norp02aa1n02x5               g059(.a(\b[14] ), .b(\a[15] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  xnrc02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  norp02aa1n02x5               g063(.a(new_n158), .b(new_n151), .o1(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(\b[13] ), .clkout(new_n160));
  oaoi03aa1n02x5               g065(.a(new_n149), .b(new_n160), .c(new_n150), .o1(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  aoai13aa1n02x5               g067(.a(new_n157), .b(new_n162), .c(new_n147), .d(new_n159), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n157), .b(new_n162), .c(new_n147), .d(new_n159), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g070(.clk(new_n155), .clkout(new_n166));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n163), .c(new_n166), .out0(\s[16] ));
  oaoi13aa1n02x5               g075(.a(new_n101), .b(new_n112), .c(new_n110), .d(new_n106), .o1(new_n171));
  norp03aa1n02x5               g076(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n172));
  nano23aa1n02x4               g077(.a(new_n155), .b(new_n167), .c(new_n168), .d(new_n156), .out0(new_n173));
  nona22aa1n02x4               g078(.a(new_n173), .b(new_n158), .c(new_n151), .out0(new_n174));
  norp02aa1n02x5               g079(.a(new_n174), .b(new_n142), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n127), .c(new_n171), .d(new_n172), .o1(new_n176));
  aoi012aa1n02x5               g081(.a(new_n167), .b(new_n155), .c(new_n168), .o1(new_n177));
  oaib12aa1n02x5               g082(.a(new_n177), .b(new_n161), .c(new_n173), .out0(new_n178));
  oab012aa1n02x4               g083(.a(new_n178), .b(new_n146), .c(new_n174), .out0(new_n179));
  nanp02aa1n02x5               g084(.a(new_n176), .b(new_n179), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[18] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\a[17] ), .clkout(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(\b[16] ), .clkout(new_n184));
  oaoi03aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n180), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(new_n182), .out0(\s[18] ));
  xroi22aa1d04x5               g091(.a(new_n183), .b(\b[16] ), .c(new_n182), .d(\b[17] ), .out0(new_n187));
  nanp02aa1n02x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  nona22aa1n02x4               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(new_n189));
  oaib12aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n182), .out0(new_n190));
  norp02aa1n02x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n192), .b(new_n191), .out0(new_n193));
  aoai13aa1n02x5               g098(.a(new_n193), .b(new_n190), .c(new_n180), .d(new_n187), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(new_n193), .b(new_n190), .c(new_n180), .d(new_n187), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  nona22aa1n02x4               g105(.a(new_n194), .b(new_n200), .c(new_n191), .out0(new_n201));
  orn002aa1n02x5               g106(.a(\a[19] ), .b(\b[18] ), .o(new_n202));
  aobi12aa1n02x5               g107(.a(new_n200), .b(new_n194), .c(new_n202), .out0(new_n203));
  norb02aa1n02x5               g108(.a(new_n201), .b(new_n203), .out0(\s[20] ));
  nano23aa1n02x4               g109(.a(new_n191), .b(new_n198), .c(new_n199), .d(new_n192), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n187), .b(new_n205), .o1(new_n206));
  norp02aa1n02x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  aoi013aa1n02x4               g112(.a(new_n207), .b(new_n188), .c(new_n183), .d(new_n184), .o1(new_n208));
  nona23aa1n02x4               g113(.a(new_n199), .b(new_n192), .c(new_n191), .d(new_n198), .out0(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[20] ), .b(\b[19] ), .c(new_n202), .o1(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  oai012aa1n02x5               g116(.a(new_n211), .b(new_n209), .c(new_n208), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n206), .c(new_n176), .d(new_n179), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  aoi112aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n214), .d(new_n217), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n218), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g126(.clk(\a[21] ), .clkout(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[22] ), .clkout(new_n223));
  xroi22aa1d04x5               g128(.a(new_n222), .b(\b[20] ), .c(new_n223), .d(\b[21] ), .out0(new_n224));
  nanp03aa1n02x5               g129(.a(new_n224), .b(new_n187), .c(new_n205), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(\b[21] ), .clkout(new_n226));
  oao003aa1n02x5               g131(.a(new_n223), .b(new_n226), .c(new_n216), .carry(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n212), .c(new_n224), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n225), .c(new_n176), .d(new_n179), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  norp02aa1n02x5               g137(.a(\b[23] ), .b(\a[24] ), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(\b[23] ), .b(\a[24] ), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n234), .b(new_n233), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n229), .d(new_n232), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  and002aa1n02x5               g143(.a(new_n232), .b(new_n235), .o(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  nano32aa1n02x4               g145(.a(new_n240), .b(new_n224), .c(new_n187), .d(new_n205), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n224), .b(new_n210), .c(new_n205), .d(new_n190), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n227), .clkout(new_n243));
  oai012aa1n02x5               g148(.a(new_n234), .b(new_n233), .c(new_n231), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n240), .c(new_n242), .d(new_n243), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[25] ), .b(\b[24] ), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n245), .c(new_n180), .d(new_n241), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n246), .b(new_n245), .c(new_n180), .d(new_n241), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(\s[25] ));
  norp02aa1n02x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  nona22aa1n02x4               g156(.a(new_n247), .b(new_n251), .c(new_n250), .out0(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n250), .clkout(new_n253));
  aobi12aa1n02x5               g158(.a(new_n251), .b(new_n247), .c(new_n253), .out0(new_n254));
  norb02aa1n02x5               g159(.a(new_n252), .b(new_n254), .out0(\s[26] ));
  nanp02aa1n02x5               g160(.a(new_n122), .b(new_n136), .o1(new_n256));
  oabi12aa1n02x5               g161(.a(new_n178), .b(new_n146), .c(new_n174), .out0(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(\a[25] ), .clkout(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(\a[26] ), .clkout(new_n259));
  xroi22aa1d04x5               g164(.a(new_n258), .b(\b[24] ), .c(new_n259), .d(\b[25] ), .out0(new_n260));
  nano22aa1n02x4               g165(.a(new_n225), .b(new_n239), .c(new_n260), .out0(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n257), .c(new_n256), .d(new_n175), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n263));
  aobi12aa1n02x5               g168(.a(new_n263), .b(new_n245), .c(new_n260), .out0(new_n264));
  xorc02aa1n02x5               g169(.a(\a[27] ), .b(\b[26] ), .out0(new_n265));
  xnbna2aa1n03x5               g170(.a(new_n265), .b(new_n264), .c(new_n262), .out0(\s[27] ));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n267), .clkout(new_n268));
  aobi12aa1n02x5               g173(.a(new_n265), .b(new_n264), .c(new_n262), .out0(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[27] ), .b(\a[28] ), .out0(new_n270));
  nano22aa1n02x4               g175(.a(new_n269), .b(new_n268), .c(new_n270), .out0(new_n271));
  aobi12aa1n02x5               g176(.a(new_n261), .b(new_n176), .c(new_n179), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n239), .b(new_n227), .c(new_n212), .d(new_n224), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n260), .clkout(new_n274));
  aoai13aa1n02x5               g179(.a(new_n263), .b(new_n274), .c(new_n273), .d(new_n244), .o1(new_n275));
  oai012aa1n02x5               g180(.a(new_n265), .b(new_n275), .c(new_n272), .o1(new_n276));
  aoi012aa1n02x5               g181(.a(new_n270), .b(new_n276), .c(new_n268), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n277), .b(new_n271), .o1(\s[28] ));
  norb02aa1n02x5               g183(.a(new_n265), .b(new_n270), .out0(new_n279));
  oai012aa1n02x5               g184(.a(new_n279), .b(new_n275), .c(new_n272), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .c(new_n268), .carry(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  aoi012aa1n02x5               g187(.a(new_n282), .b(new_n280), .c(new_n281), .o1(new_n283));
  aobi12aa1n02x5               g188(.a(new_n279), .b(new_n264), .c(new_n262), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n284), .b(new_n281), .c(new_n282), .out0(new_n285));
  norp02aa1n02x5               g190(.a(new_n283), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g191(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g192(.a(new_n265), .b(new_n282), .c(new_n270), .out0(new_n288));
  oai012aa1n02x5               g193(.a(new_n288), .b(new_n275), .c(new_n272), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[29] ), .b(\b[28] ), .c(new_n281), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  aoi012aa1n02x5               g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x5               g197(.a(new_n288), .b(new_n264), .c(new_n262), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g200(.a(new_n288), .b(new_n291), .out0(new_n296));
  aobi12aa1n02x5               g201(.a(new_n296), .b(new_n264), .c(new_n262), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[30] ), .b(\b[29] ), .c(new_n290), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[30] ), .b(\a[31] ), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  oai012aa1n02x5               g205(.a(new_n296), .b(new_n275), .c(new_n272), .o1(new_n301));
  aoi012aa1n02x5               g206(.a(new_n299), .b(new_n301), .c(new_n298), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n300), .o1(\s[31] ));
  xnbna2aa1n03x5               g208(.a(new_n110), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g209(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g211(.a(new_n171), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g212(.a(\a[5] ), .b(\b[4] ), .o(new_n308));
  nona22aa1n02x4               g213(.a(new_n113), .b(new_n120), .c(new_n101), .out0(new_n309));
  xobna2aa1n03x5               g214(.a(new_n119), .b(new_n309), .c(new_n308), .out0(\s[6] ));
  aobi12aa1n02x5               g215(.a(new_n125), .b(new_n171), .c(new_n121), .out0(new_n311));
  xnrb03aa1n02x5               g216(.a(new_n311), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g217(.a(\a[7] ), .b(\b[6] ), .c(new_n311), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g219(.a(new_n128), .b(new_n122), .c(new_n136), .out0(\s[9] ));
endmodule


