// Benchmark "adder" written by ABC on Thu Jul 11 12:32:34 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n322, new_n324, new_n326;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  160nm_ficinv00aa1n08x5       g008(.clk(\a[3] ), .clkout(new_n104));
  nanb02aa1n02x5               g009(.a(\b[2] ), .b(new_n104), .out0(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  oao003aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n105), .carry(new_n108));
  oai013aa1n02x4               g013(.a(new_n108), .b(new_n103), .c(new_n102), .d(new_n107), .o1(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n02x4               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n115), .b(new_n116), .c(new_n110), .o1(new_n117));
  aoi112aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(new_n118), .clkout(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  aob012aa1n02x5               g025(.a(new_n120), .b(\b[5] ), .c(\a[6] ), .out0(new_n121));
  oai122aa1n02x7               g026(.a(new_n119), .b(new_n115), .c(new_n121), .d(\b[7] ), .e(\a[8] ), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n02x5               g028(.a(new_n123), .b(new_n97), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g034(.a(new_n109), .b(new_n117), .o1(new_n130));
  nano23aa1n02x4               g035(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n131));
  160nm_ficinv00aa1n08x5       g036(.clk(\a[5] ), .clkout(new_n132));
  160nm_ficinv00aa1n08x5       g037(.clk(\b[4] ), .clkout(new_n133));
  nanp02aa1n02x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  oaoi03aa1n02x5               g039(.a(\a[6] ), .b(\b[5] ), .c(new_n134), .o1(new_n135));
  aoi112aa1n02x5               g040(.a(new_n118), .b(new_n111), .c(new_n131), .d(new_n135), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(new_n130), .b(new_n136), .o1(new_n137));
  norp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  oai012aa1n02x5               g045(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n141));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n141), .clkout(new_n142));
  nano23aa1n02x4               g047(.a(new_n97), .b(new_n126), .c(new_n127), .d(new_n123), .out0(new_n143));
  aoai13aa1n02x5               g048(.a(new_n140), .b(new_n142), .c(new_n137), .d(new_n143), .o1(new_n144));
  aoi112aa1n02x5               g049(.a(new_n142), .b(new_n140), .c(new_n137), .d(new_n143), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n144), .b(new_n145), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n138), .clkout(new_n147));
  norp02aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  xnbna2aa1n03x5               g055(.a(new_n150), .b(new_n144), .c(new_n147), .out0(\s[12] ));
  nona23aa1n02x4               g056(.a(new_n149), .b(new_n139), .c(new_n138), .d(new_n148), .out0(new_n152));
  nano22aa1n02x4               g057(.a(new_n152), .b(new_n124), .c(new_n128), .out0(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n154));
  oaoi03aa1n02x5               g059(.a(\a[12] ), .b(\b[11] ), .c(new_n147), .o1(new_n155));
  oabi12aa1n02x5               g060(.a(new_n155), .b(new_n152), .c(new_n141), .out0(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n156), .clkout(new_n157));
  xorc02aa1n02x5               g062(.a(\a[13] ), .b(\b[12] ), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n154), .c(new_n157), .out0(\s[13] ));
  orn002aa1n02x5               g064(.a(\a[13] ), .b(\b[12] ), .o(new_n160));
  xnrc02aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .out0(new_n161));
  aoai13aa1n02x5               g066(.a(new_n160), .b(new_n161), .c(new_n154), .d(new_n157), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .out0(new_n164));
  nanb02aa1n02x5               g069(.a(new_n164), .b(new_n158), .out0(new_n165));
  oaoi03aa1n02x5               g070(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .o1(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n166), .clkout(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n165), .c(new_n154), .d(new_n157), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  xorc02aa1n02x5               g075(.a(\a[15] ), .b(\b[14] ), .out0(new_n171));
  xorc02aa1n02x5               g076(.a(\a[16] ), .b(\b[15] ), .out0(new_n172));
  aoi112aa1n02x5               g077(.a(new_n172), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  nano23aa1n02x4               g080(.a(new_n138), .b(new_n148), .c(new_n149), .d(new_n139), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n143), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(new_n172), .b(new_n171), .o1(new_n178));
  norp03aa1n02x5               g083(.a(new_n177), .b(new_n165), .c(new_n178), .o1(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n180));
  norp02aa1n02x5               g085(.a(new_n164), .b(new_n161), .o1(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(new_n178), .clkout(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n166), .c(new_n156), .d(new_n181), .o1(new_n183));
  aoi112aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n184));
  oab012aa1n02x4               g089(.a(new_n184), .b(\a[16] ), .c(\b[15] ), .out0(new_n185));
  nanp03aa1n02x5               g090(.a(new_n180), .b(new_n183), .c(new_n185), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[18] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[17] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\b[16] ), .clkout(new_n190));
  oaoi03aa1n02x5               g095(.a(new_n189), .b(new_n190), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  nanp03aa1n02x5               g097(.a(new_n182), .b(new_n153), .c(new_n181), .o1(new_n193));
  aoi012aa1n02x5               g098(.a(new_n193), .b(new_n130), .c(new_n136), .o1(new_n194));
  aoai13aa1n02x5               g099(.a(new_n181), .b(new_n155), .c(new_n176), .d(new_n142), .o1(new_n195));
  aoai13aa1n02x5               g100(.a(new_n185), .b(new_n178), .c(new_n195), .d(new_n167), .o1(new_n196));
  xroi22aa1d04x5               g101(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n197));
  oai012aa1n02x5               g102(.a(new_n197), .b(new_n196), .c(new_n194), .o1(new_n198));
  oai022aa1n02x5               g103(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n199));
  oaib12aa1n02x5               g104(.a(new_n199), .b(new_n188), .c(\b[17] ), .out0(new_n200));
  norp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n203), .clkout(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n198), .c(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n201), .clkout(new_n207));
  aoi012aa1n02x5               g112(.a(new_n203), .b(new_n198), .c(new_n200), .o1(new_n208));
  norp02aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanp02aa1n02x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  nano22aa1n02x4               g116(.a(new_n208), .b(new_n207), .c(new_n211), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n190), .b(new_n189), .o1(new_n213));
  oaoi03aa1n02x5               g118(.a(\a[18] ), .b(\b[17] ), .c(new_n213), .o1(new_n214));
  aoai13aa1n02x5               g119(.a(new_n204), .b(new_n214), .c(new_n186), .d(new_n197), .o1(new_n215));
  aoi012aa1n02x5               g120(.a(new_n211), .b(new_n215), .c(new_n207), .o1(new_n216));
  norp02aa1n02x5               g121(.a(new_n216), .b(new_n212), .o1(\s[20] ));
  nano23aa1n02x4               g122(.a(new_n201), .b(new_n209), .c(new_n210), .d(new_n202), .out0(new_n218));
  nanp02aa1n02x5               g123(.a(new_n197), .b(new_n218), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  oai012aa1n02x5               g125(.a(new_n220), .b(new_n196), .c(new_n194), .o1(new_n221));
  nona23aa1n02x4               g126(.a(new_n210), .b(new_n202), .c(new_n201), .d(new_n209), .out0(new_n222));
  aoi012aa1n02x5               g127(.a(new_n209), .b(new_n201), .c(new_n210), .o1(new_n223));
  oai012aa1n02x5               g128(.a(new_n223), .b(new_n222), .c(new_n200), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n224), .clkout(new_n225));
  norp02aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  nanp02aa1n02x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(new_n228));
  xnbna2aa1n03x5               g133(.a(new_n228), .b(new_n221), .c(new_n225), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n226), .clkout(new_n230));
  aobi12aa1n02x5               g135(.a(new_n228), .b(new_n221), .c(new_n225), .out0(new_n231));
  xnrc02aa1n02x5               g136(.a(\b[21] ), .b(\a[22] ), .out0(new_n232));
  nano22aa1n02x4               g137(.a(new_n231), .b(new_n230), .c(new_n232), .out0(new_n233));
  aoai13aa1n02x5               g138(.a(new_n228), .b(new_n224), .c(new_n186), .d(new_n220), .o1(new_n234));
  aoi012aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .o1(new_n235));
  norp02aa1n02x5               g140(.a(new_n235), .b(new_n233), .o1(\s[22] ));
  nano22aa1n02x4               g141(.a(new_n232), .b(new_n230), .c(new_n227), .out0(new_n237));
  and003aa1n02x5               g142(.a(new_n197), .b(new_n237), .c(new_n218), .o(new_n238));
  oai012aa1n02x5               g143(.a(new_n238), .b(new_n196), .c(new_n194), .o1(new_n239));
  oao003aa1n02x5               g144(.a(\a[22] ), .b(\b[21] ), .c(new_n230), .carry(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  aoi012aa1n02x5               g146(.a(new_n241), .b(new_n224), .c(new_n237), .o1(new_n242));
  xnrc02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .out0(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  xnbna2aa1n03x5               g149(.a(new_n244), .b(new_n239), .c(new_n242), .out0(\s[23] ));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  aoi012aa1n02x5               g152(.a(new_n243), .b(new_n239), .c(new_n242), .o1(new_n248));
  xnrc02aa1n02x5               g153(.a(\b[23] ), .b(\a[24] ), .out0(new_n249));
  nano22aa1n02x4               g154(.a(new_n248), .b(new_n247), .c(new_n249), .out0(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n242), .clkout(new_n251));
  aoai13aa1n02x5               g156(.a(new_n244), .b(new_n251), .c(new_n186), .d(new_n238), .o1(new_n252));
  aoi012aa1n02x5               g157(.a(new_n249), .b(new_n252), .c(new_n247), .o1(new_n253));
  norp02aa1n02x5               g158(.a(new_n253), .b(new_n250), .o1(\s[24] ));
  norp02aa1n02x5               g159(.a(new_n249), .b(new_n243), .o1(new_n255));
  nano22aa1n02x4               g160(.a(new_n219), .b(new_n237), .c(new_n255), .out0(new_n256));
  oai012aa1n02x5               g161(.a(new_n256), .b(new_n196), .c(new_n194), .o1(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n223), .clkout(new_n258));
  aoai13aa1n02x5               g163(.a(new_n237), .b(new_n258), .c(new_n218), .d(new_n214), .o1(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n255), .clkout(new_n260));
  oao003aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n247), .carry(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n260), .c(new_n259), .d(new_n240), .o1(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .out0(new_n263));
  aoib12aa1n02x5               g168(.a(new_n263), .b(new_n257), .c(new_n262), .out0(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n263), .clkout(new_n265));
  aoi112aa1n02x5               g170(.a(new_n265), .b(new_n262), .c(new_n186), .d(new_n256), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n264), .b(new_n266), .o1(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n268), .clkout(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[25] ), .b(\a[26] ), .out0(new_n270));
  nano22aa1n02x4               g175(.a(new_n264), .b(new_n269), .c(new_n270), .out0(new_n271));
  aoai13aa1n02x5               g176(.a(new_n265), .b(new_n262), .c(new_n186), .d(new_n256), .o1(new_n272));
  aoi012aa1n02x5               g177(.a(new_n270), .b(new_n272), .c(new_n269), .o1(new_n273));
  norp02aa1n02x5               g178(.a(new_n273), .b(new_n271), .o1(\s[26] ));
  norp02aa1n02x5               g179(.a(new_n270), .b(new_n263), .o1(new_n275));
  nano32aa1n02x4               g180(.a(new_n219), .b(new_n275), .c(new_n237), .d(new_n255), .out0(new_n276));
  oai012aa1n02x5               g181(.a(new_n276), .b(new_n196), .c(new_n194), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n278));
  aobi12aa1n02x5               g183(.a(new_n278), .b(new_n262), .c(new_n275), .out0(new_n279));
  xorc02aa1n02x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n279), .out0(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n282), .clkout(new_n283));
  aobi12aa1n02x5               g188(.a(new_n280), .b(new_n277), .c(new_n279), .out0(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n284), .b(new_n283), .c(new_n285), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n255), .b(new_n241), .c(new_n224), .d(new_n237), .o1(new_n287));
  160nm_ficinv00aa1n08x5       g192(.clk(new_n275), .clkout(new_n288));
  aoai13aa1n02x5               g193(.a(new_n278), .b(new_n288), .c(new_n287), .d(new_n261), .o1(new_n289));
  aoai13aa1n02x5               g194(.a(new_n280), .b(new_n289), .c(new_n186), .d(new_n276), .o1(new_n290));
  aoi012aa1n02x5               g195(.a(new_n285), .b(new_n290), .c(new_n283), .o1(new_n291));
  norp02aa1n02x5               g196(.a(new_n291), .b(new_n286), .o1(\s[28] ));
  norb02aa1n02x5               g197(.a(new_n280), .b(new_n285), .out0(new_n293));
  aobi12aa1n02x5               g198(.a(new_n293), .b(new_n277), .c(new_n279), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[28] ), .b(\a[29] ), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n297));
  aoai13aa1n02x5               g202(.a(new_n293), .b(new_n289), .c(new_n186), .d(new_n276), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n296), .b(new_n298), .c(new_n295), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n299), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g206(.a(new_n280), .b(new_n296), .c(new_n285), .out0(new_n302));
  aobi12aa1n02x5               g207(.a(new_n302), .b(new_n277), .c(new_n279), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  aoai13aa1n02x5               g211(.a(new_n302), .b(new_n289), .c(new_n186), .d(new_n276), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n306), .o1(\s[30] ));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  norb02aa1n02x5               g215(.a(new_n302), .b(new_n305), .out0(new_n311));
  aobi12aa1n02x5               g216(.a(new_n311), .b(new_n277), .c(new_n279), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n313));
  nano22aa1n02x4               g218(.a(new_n312), .b(new_n310), .c(new_n313), .out0(new_n314));
  aoai13aa1n02x5               g219(.a(new_n311), .b(new_n289), .c(new_n186), .d(new_n276), .o1(new_n315));
  aoi012aa1n02x5               g220(.a(new_n310), .b(new_n315), .c(new_n313), .o1(new_n316));
  norp02aa1n02x5               g221(.a(new_n316), .b(new_n314), .o1(\s[31] ));
  xnbna2aa1n03x5               g222(.a(new_n102), .b(new_n106), .c(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g223(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g225(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaib12aa1n02x5               g226(.a(new_n134), .b(new_n116), .c(new_n109), .out0(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoib12aa1n02x5               g228(.a(new_n135), .b(new_n322), .c(new_n110), .out0(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g230(.a(\a[7] ), .b(\b[6] ), .c(new_n324), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g232(.a(new_n124), .b(new_n130), .c(new_n136), .out0(\s[9] ));
endmodule


