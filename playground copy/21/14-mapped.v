// Benchmark "adder" written by ABC on Thu Jul 11 12:26:03 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n123, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n146, new_n148, new_n149, new_n150,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n311, new_n313,
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
  oai012aa1n02x5               g010(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  norp03aa1n02x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  aoi112aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n116));
  oab012aa1n02x4               g021(.a(new_n116), .b(\a[6] ), .c(\b[5] ), .out0(new_n117));
  160nm_fiao0012aa1n02p5x5     g022(.a(new_n108), .b(new_n110), .c(new_n109), .o(new_n118));
  oabi12aa1n02x5               g023(.a(new_n118), .b(new_n112), .c(new_n117), .out0(new_n119));
  aoi012aa1n02x5               g024(.a(new_n119), .b(new_n107), .c(new_n115), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[9] ), .b(\b[8] ), .c(new_n120), .o1(new_n121));
  xorb03aa1n02x5               g026(.a(new_n121), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g027(.a(\b[10] ), .b(\a[11] ), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(\b[10] ), .b(\a[11] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n123), .out0(new_n125));
  norp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nano23aa1n02x4               g034(.a(new_n126), .b(new_n128), .c(new_n129), .d(new_n127), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n119), .c(new_n107), .d(new_n115), .o1(new_n131));
  aoi012aa1n02x5               g036(.a(new_n128), .b(new_n126), .c(new_n129), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n125), .b(new_n131), .c(new_n132), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n123), .clkout(new_n134));
  aob012aa1n02x5               g039(.a(new_n125), .b(new_n131), .c(new_n132), .out0(new_n135));
  norp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n135), .c(new_n134), .out0(\s[12] ));
  160nm_ficinv00aa1n08x5       g044(.clk(\a[13] ), .clkout(new_n140));
  nona23aa1n02x4               g045(.a(new_n137), .b(new_n124), .c(new_n123), .d(new_n136), .out0(new_n141));
  oaoi03aa1n02x5               g046(.a(\a[12] ), .b(\b[11] ), .c(new_n134), .o1(new_n142));
  oabi12aa1n02x5               g047(.a(new_n142), .b(new_n141), .c(new_n132), .out0(new_n143));
  oab012aa1n02x4               g048(.a(new_n143), .b(new_n131), .c(new_n141), .out0(new_n144));
  xorb03aa1n02x5               g049(.a(new_n144), .b(\b[12] ), .c(new_n140), .out0(\s[13] ));
  oaoi03aa1n02x5               g050(.a(\a[13] ), .b(\b[12] ), .c(new_n144), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nano23aa1n02x4               g052(.a(new_n123), .b(new_n136), .c(new_n137), .d(new_n124), .out0(new_n148));
  norp02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  norp02aa1n02x5               g055(.a(\b[13] ), .b(\a[14] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  nano23aa1n02x4               g057(.a(new_n149), .b(new_n151), .c(new_n152), .d(new_n150), .out0(new_n153));
  nano32aa1n02x4               g058(.a(new_n120), .b(new_n153), .c(new_n130), .d(new_n148), .out0(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n132), .clkout(new_n155));
  aoai13aa1n02x5               g060(.a(new_n153), .b(new_n142), .c(new_n148), .d(new_n155), .o1(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(\b[12] ), .clkout(new_n157));
  aoai13aa1n02x5               g062(.a(new_n152), .b(new_n151), .c(new_n140), .d(new_n157), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(new_n156), .b(new_n158), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  nanp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  oai112aa1n02x5               g067(.a(new_n161), .b(new_n162), .c(new_n154), .d(new_n159), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n154), .b(new_n159), .c(new_n162), .d(new_n161), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(\s[15] ));
  norp02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n163), .c(new_n161), .out0(\s[16] ));
  nano23aa1n02x4               g074(.a(new_n160), .b(new_n166), .c(new_n167), .d(new_n162), .out0(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(new_n170), .clkout(new_n171));
  nano32aa1n02x4               g076(.a(new_n171), .b(new_n153), .c(new_n148), .d(new_n130), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n119), .c(new_n107), .d(new_n115), .o1(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n158), .clkout(new_n174));
  aoai13aa1n02x5               g079(.a(new_n170), .b(new_n174), .c(new_n143), .d(new_n153), .o1(new_n175));
  oai012aa1n02x5               g080(.a(new_n167), .b(new_n166), .c(new_n160), .o1(new_n176));
  nanp03aa1n02x5               g081(.a(new_n173), .b(new_n175), .c(new_n176), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  norp02aa1n02x5               g083(.a(\b[16] ), .b(\a[17] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[16] ), .b(\a[17] ), .o1(new_n180));
  aoi012aa1n02x5               g085(.a(new_n179), .b(new_n177), .c(new_n180), .o1(new_n181));
  xnrb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  aobi12aa1n02x5               g087(.a(new_n176), .b(new_n159), .c(new_n170), .out0(new_n183));
  norp02aa1n02x5               g088(.a(\b[17] ), .b(\a[18] ), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(\b[17] ), .b(\a[18] ), .o1(new_n185));
  nano23aa1n02x4               g090(.a(new_n179), .b(new_n184), .c(new_n185), .d(new_n180), .out0(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(new_n186), .clkout(new_n187));
  aoi012aa1n02x5               g092(.a(new_n184), .b(new_n179), .c(new_n185), .o1(new_n188));
  aoai13aa1n02x5               g093(.a(new_n188), .b(new_n187), .c(new_n183), .d(new_n173), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g095(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  norp02aa1n02x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoi112aa1n02x5               g102(.a(new_n192), .b(new_n197), .c(new_n189), .d(new_n194), .o1(new_n198));
  160nm_ficinv00aa1n08x5       g103(.clk(new_n192), .clkout(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n188), .clkout(new_n200));
  aoai13aa1n02x5               g105(.a(new_n194), .b(new_n200), .c(new_n177), .d(new_n186), .o1(new_n201));
  aobi12aa1n02x5               g106(.a(new_n197), .b(new_n201), .c(new_n199), .out0(new_n202));
  norp02aa1n02x5               g107(.a(new_n202), .b(new_n198), .o1(\s[20] ));
  nano23aa1n02x4               g108(.a(new_n192), .b(new_n195), .c(new_n196), .d(new_n193), .out0(new_n204));
  nanp02aa1n02x5               g109(.a(new_n204), .b(new_n186), .o1(new_n205));
  nona23aa1n02x4               g110(.a(new_n196), .b(new_n193), .c(new_n192), .d(new_n195), .out0(new_n206));
  oaoi03aa1n02x5               g111(.a(\a[20] ), .b(\b[19] ), .c(new_n199), .o1(new_n207));
  160nm_ficinv00aa1n08x5       g112(.clk(new_n207), .clkout(new_n208));
  oai012aa1n02x5               g113(.a(new_n208), .b(new_n206), .c(new_n188), .o1(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  aoai13aa1n02x5               g115(.a(new_n210), .b(new_n205), .c(new_n183), .d(new_n173), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g117(.a(\b[20] ), .b(\a[21] ), .o1(new_n213));
  xorc02aa1n02x5               g118(.a(\a[21] ), .b(\b[20] ), .out0(new_n214));
  xorc02aa1n02x5               g119(.a(\a[22] ), .b(\b[21] ), .out0(new_n215));
  aoi112aa1n02x5               g120(.a(new_n213), .b(new_n215), .c(new_n211), .d(new_n214), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n213), .clkout(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n205), .clkout(new_n218));
  aoai13aa1n02x5               g123(.a(new_n214), .b(new_n209), .c(new_n177), .d(new_n218), .o1(new_n219));
  aobi12aa1n02x5               g124(.a(new_n215), .b(new_n219), .c(new_n217), .out0(new_n220));
  norp02aa1n02x5               g125(.a(new_n220), .b(new_n216), .o1(\s[22] ));
  nano22aa1n02x4               g126(.a(new_n205), .b(new_n214), .c(new_n215), .out0(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n222), .clkout(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[21] ), .clkout(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\a[22] ), .clkout(new_n225));
  xroi22aa1d04x5               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  oaoi03aa1n02x5               g131(.a(\a[22] ), .b(\b[21] ), .c(new_n217), .o1(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n209), .c(new_n226), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n223), .c(new_n183), .d(new_n173), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xorc02aa1n02x5               g137(.a(\a[24] ), .b(\b[23] ), .out0(new_n233));
  aoi112aa1n02x5               g138(.a(new_n231), .b(new_n233), .c(new_n229), .d(new_n232), .o1(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n231), .clkout(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n228), .clkout(new_n236));
  aoai13aa1n02x5               g141(.a(new_n232), .b(new_n236), .c(new_n177), .d(new_n222), .o1(new_n237));
  aobi12aa1n02x5               g142(.a(new_n233), .b(new_n237), .c(new_n235), .out0(new_n238));
  norp02aa1n02x5               g143(.a(new_n238), .b(new_n234), .o1(\s[24] ));
  and002aa1n02x5               g144(.a(new_n233), .b(new_n232), .o(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  nano32aa1n02x4               g146(.a(new_n241), .b(new_n226), .c(new_n204), .d(new_n186), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  aoai13aa1n02x5               g148(.a(new_n226), .b(new_n207), .c(new_n204), .d(new_n200), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n227), .clkout(new_n245));
  nanp02aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  oai022aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(new_n247), .b(new_n246), .o1(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n241), .c(new_n244), .d(new_n245), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n243), .c(new_n183), .d(new_n173), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xorc02aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  aoi112aa1n02x5               g160(.a(new_n253), .b(new_n255), .c(new_n251), .d(new_n254), .o1(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n253), .clkout(new_n257));
  aoai13aa1n02x5               g162(.a(new_n254), .b(new_n249), .c(new_n177), .d(new_n242), .o1(new_n258));
  aobi12aa1n02x5               g163(.a(new_n255), .b(new_n258), .c(new_n257), .out0(new_n259));
  norp02aa1n02x5               g164(.a(new_n259), .b(new_n256), .o1(\s[26] ));
  norb02aa1n02x5               g165(.a(new_n172), .b(new_n120), .out0(new_n261));
  aoai13aa1n02x5               g166(.a(new_n176), .b(new_n171), .c(new_n156), .d(new_n158), .o1(new_n262));
  and002aa1n02x5               g167(.a(new_n255), .b(new_n254), .o(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  nano23aa1n02x4               g169(.a(new_n205), .b(new_n264), .c(new_n240), .d(new_n226), .out0(new_n265));
  oai012aa1n02x5               g170(.a(new_n265), .b(new_n261), .c(new_n262), .o1(new_n266));
  oao003aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .c(new_n257), .carry(new_n267));
  aobi12aa1n02x5               g172(.a(new_n267), .b(new_n249), .c(new_n263), .out0(new_n268));
  xorc02aa1n02x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n266), .c(new_n268), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n271), .clkout(new_n272));
  aobi12aa1n02x5               g177(.a(new_n269), .b(new_n266), .c(new_n268), .out0(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  nano22aa1n02x4               g179(.a(new_n273), .b(new_n272), .c(new_n274), .out0(new_n275));
  aoai13aa1n02x5               g180(.a(new_n240), .b(new_n227), .c(new_n209), .d(new_n226), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n267), .b(new_n264), .c(new_n276), .d(new_n248), .o1(new_n277));
  aoai13aa1n02x5               g182(.a(new_n269), .b(new_n277), .c(new_n177), .d(new_n265), .o1(new_n278));
  aoi012aa1n02x5               g183(.a(new_n274), .b(new_n278), .c(new_n272), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n279), .b(new_n275), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n269), .b(new_n274), .out0(new_n281));
  aobi12aa1n02x5               g186(.a(new_n281), .b(new_n266), .c(new_n268), .out0(new_n282));
  oao003aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n272), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n282), .b(new_n283), .c(new_n284), .out0(new_n285));
  aoai13aa1n02x5               g190(.a(new_n281), .b(new_n277), .c(new_n177), .d(new_n265), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n284), .b(new_n286), .c(new_n283), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n269), .b(new_n284), .c(new_n274), .out0(new_n290));
  aobi12aa1n02x5               g195(.a(new_n290), .b(new_n266), .c(new_n268), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n291), .b(new_n292), .c(new_n293), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n290), .b(new_n277), .c(new_n177), .d(new_n265), .o1(new_n295));
  aoi012aa1n02x5               g200(.a(new_n293), .b(new_n295), .c(new_n292), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n296), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g202(.a(new_n290), .b(new_n293), .out0(new_n298));
  aobi12aa1n02x5               g203(.a(new_n298), .b(new_n266), .c(new_n268), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n299), .b(new_n300), .c(new_n301), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n298), .b(new_n277), .c(new_n177), .d(new_n265), .o1(new_n303));
  aoi012aa1n02x5               g208(.a(new_n301), .b(new_n303), .c(new_n300), .o1(new_n304));
  norp02aa1n02x5               g209(.a(new_n304), .b(new_n302), .o1(\s[31] ));
  xnrb03aa1n02x5               g210(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g211(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g213(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g214(.a(\a[5] ), .b(\b[4] ), .o(new_n310));
  nanb02aa1n02x5               g215(.a(new_n114), .b(new_n107), .out0(new_n311));
  xobna2aa1n03x5               g216(.a(new_n113), .b(new_n311), .c(new_n310), .out0(\s[6] ));
  norp02aa1n02x5               g217(.a(new_n114), .b(new_n113), .o1(new_n313));
  aobi12aa1n02x5               g218(.a(new_n117), .b(new_n107), .c(new_n313), .out0(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g220(.a(\a[7] ), .b(\b[6] ), .c(new_n314), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g222(.a(new_n120), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


