// Benchmark "adder" written by ABC on Thu Jul 11 12:56:51 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n314, new_n316;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  and002aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[3] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\b[2] ), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  aoi012aa1n02x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  oa0022aa1n02x5               g013(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n109));
  oaoi13aa1n02x5               g014(.a(new_n99), .b(new_n109), .c(new_n108), .d(new_n104), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n02x4               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  norp03aa1n02x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  aoi112aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n120));
  orn002aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .o(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[6] ), .b(\b[5] ), .c(new_n121), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(new_n120), .b(new_n122), .o1(new_n123));
  nona22aa1n02x4               g028(.a(new_n123), .b(new_n119), .c(new_n111), .out0(new_n124));
  xorc02aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n97), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  aobi12aa1n02x5               g032(.a(new_n97), .b(new_n126), .c(new_n98), .out0(new_n128));
  norp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  oai022aa1n02x5               g036(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n132));
  aob012aa1n02x5               g037(.a(new_n132), .b(\b[9] ), .c(\a[10] ), .out0(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n133), .clkout(new_n134));
  oai012aa1n02x5               g039(.a(new_n131), .b(new_n128), .c(new_n134), .o1(new_n135));
  norp03aa1n02x5               g040(.a(new_n128), .b(new_n131), .c(new_n134), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  norp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  nona22aa1n02x4               g045(.a(new_n135), .b(new_n140), .c(new_n129), .out0(new_n141));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n129), .clkout(new_n142));
  aobi12aa1n02x5               g047(.a(new_n140), .b(new_n135), .c(new_n142), .out0(new_n143));
  norb02aa1n02x5               g048(.a(new_n141), .b(new_n143), .out0(\s[12] ));
  nano23aa1n02x4               g049(.a(new_n129), .b(new_n138), .c(new_n139), .d(new_n130), .out0(new_n145));
  nanp03aa1n02x5               g050(.a(new_n145), .b(new_n97), .c(new_n125), .o1(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n146), .clkout(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n148));
  nona23aa1n02x4               g053(.a(new_n139), .b(new_n130), .c(new_n129), .d(new_n138), .out0(new_n149));
  oai012aa1n02x5               g054(.a(new_n139), .b(new_n138), .c(new_n129), .o1(new_n150));
  oai012aa1n02x5               g055(.a(new_n150), .b(new_n149), .c(new_n133), .o1(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(new_n151), .clkout(new_n152));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  xobna2aa1n03x5               g060(.a(new_n155), .b(new_n148), .c(new_n152), .out0(\s[13] ));
  oai012aa1n02x5               g061(.a(new_n109), .b(new_n108), .c(new_n104), .o1(new_n157));
  norp02aa1n02x5               g062(.a(new_n117), .b(new_n116), .o1(new_n158));
  nona23aa1n02x4               g063(.a(new_n157), .b(new_n158), .c(new_n115), .d(new_n99), .out0(new_n159));
  aoi112aa1n02x5               g064(.a(new_n119), .b(new_n111), .c(new_n120), .d(new_n122), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n152), .b(new_n146), .c(new_n159), .d(new_n160), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n153), .b(new_n161), .c(new_n154), .o1(new_n162));
  xnrb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nona23aa1n02x4               g070(.a(new_n165), .b(new_n154), .c(new_n153), .d(new_n164), .out0(new_n166));
  oai012aa1n02x5               g071(.a(new_n165), .b(new_n164), .c(new_n153), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n166), .c(new_n148), .d(new_n152), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  norp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  160nm_ficinv00aa1n08x5       g077(.clk(new_n172), .clkout(new_n173));
  nanp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  aoi122aa1n02x5               g079(.a(new_n170), .b(new_n174), .c(new_n173), .d(new_n168), .e(new_n171), .o1(new_n175));
  aoi012aa1n02x5               g080(.a(new_n170), .b(new_n168), .c(new_n171), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n172), .b(new_n174), .out0(new_n177));
  norp02aa1n02x5               g082(.a(new_n176), .b(new_n177), .o1(new_n178));
  norp02aa1n02x5               g083(.a(new_n178), .b(new_n175), .o1(\s[16] ));
  nano23aa1n02x4               g084(.a(new_n153), .b(new_n164), .c(new_n165), .d(new_n154), .out0(new_n180));
  nano23aa1n02x4               g085(.a(new_n170), .b(new_n172), .c(new_n174), .d(new_n171), .out0(new_n181));
  nano22aa1n02x4               g086(.a(new_n146), .b(new_n180), .c(new_n181), .out0(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n170), .b(new_n171), .out0(new_n184));
  norp03aa1n02x5               g089(.a(new_n166), .b(new_n177), .c(new_n184), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n186));
  oai013aa1n02x4               g091(.a(new_n173), .b(new_n167), .c(new_n184), .d(new_n177), .o1(new_n187));
  aoi112aa1n02x5               g092(.a(new_n187), .b(new_n186), .c(new_n151), .d(new_n185), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n183), .b(new_n188), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g095(.clk(\a[18] ), .clkout(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(\a[17] ), .clkout(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(\b[16] ), .clkout(new_n193));
  oaoi03aa1n02x5               g098(.a(new_n192), .b(new_n193), .c(new_n189), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[17] ), .c(new_n191), .out0(\s[18] ));
  xroi22aa1d04x5               g100(.a(new_n192), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n196));
  nanp02aa1n02x5               g101(.a(new_n193), .b(new_n192), .o1(new_n197));
  oaoi03aa1n02x5               g102(.a(\a[18] ), .b(\b[17] ), .c(new_n197), .o1(new_n198));
  norp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoai13aa1n02x5               g106(.a(new_n201), .b(new_n198), .c(new_n189), .d(new_n196), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n201), .b(new_n198), .c(new_n189), .d(new_n196), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  nona22aa1n02x4               g113(.a(new_n202), .b(new_n208), .c(new_n199), .out0(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n208), .clkout(new_n210));
  oaoi13aa1n02x5               g115(.a(new_n210), .b(new_n202), .c(\a[19] ), .d(\b[18] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n209), .b(new_n211), .out0(\s[20] ));
  nano23aa1n02x4               g117(.a(new_n199), .b(new_n206), .c(new_n207), .d(new_n200), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n196), .b(new_n213), .o1(new_n214));
  oai022aa1n02x5               g119(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n215));
  oaib12aa1n02x5               g120(.a(new_n215), .b(new_n191), .c(\b[17] ), .out0(new_n216));
  nona23aa1n02x4               g121(.a(new_n207), .b(new_n200), .c(new_n199), .d(new_n206), .out0(new_n217));
  aoi012aa1n02x5               g122(.a(new_n206), .b(new_n199), .c(new_n207), .o1(new_n218));
  oai012aa1n02x5               g123(.a(new_n218), .b(new_n217), .c(new_n216), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n214), .c(new_n183), .d(new_n188), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g133(.clk(\a[21] ), .clkout(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[22] ), .clkout(new_n230));
  xroi22aa1d04x5               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  nanp03aa1n02x5               g136(.a(new_n231), .b(new_n196), .c(new_n213), .o1(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(\b[21] ), .clkout(new_n233));
  oaoi03aa1n02x5               g138(.a(new_n230), .b(new_n233), .c(new_n223), .o1(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n234), .clkout(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n219), .c(new_n231), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n232), .c(new_n183), .d(new_n188), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoi112aa1n02x5               g146(.a(new_n239), .b(new_n241), .c(new_n237), .d(new_n240), .o1(new_n242));
  aoai13aa1n02x5               g147(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  and002aa1n02x5               g149(.a(new_n241), .b(new_n240), .o(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n245), .clkout(new_n246));
  nano32aa1n02x4               g151(.a(new_n246), .b(new_n231), .c(new_n196), .d(new_n213), .out0(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n218), .clkout(new_n248));
  aoai13aa1n02x5               g153(.a(new_n231), .b(new_n248), .c(new_n213), .d(new_n198), .o1(new_n249));
  norp02aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  nanp02aa1n02x5               g155(.a(\b[23] ), .b(\a[24] ), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n250), .b(new_n239), .c(new_n251), .o1(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n246), .c(new_n249), .d(new_n234), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n253), .c(new_n189), .d(new_n247), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n254), .b(new_n253), .c(new_n189), .d(new_n247), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n255), .b(new_n256), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  nona22aa1n02x4               g164(.a(new_n255), .b(new_n259), .c(new_n258), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n258), .clkout(new_n261));
  aobi12aa1n02x5               g166(.a(new_n259), .b(new_n255), .c(new_n261), .out0(new_n262));
  norb02aa1n02x5               g167(.a(new_n260), .b(new_n262), .out0(\s[26] ));
  and002aa1n02x5               g168(.a(new_n259), .b(new_n254), .o(new_n264));
  nano22aa1n02x4               g169(.a(new_n232), .b(new_n245), .c(new_n264), .out0(new_n265));
  nanp02aa1n02x5               g170(.a(new_n189), .b(new_n265), .o1(new_n266));
  oao003aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n267));
  aobi12aa1n02x5               g172(.a(new_n267), .b(new_n253), .c(new_n264), .out0(new_n268));
  xorc02aa1n02x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n268), .c(new_n266), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n271), .clkout(new_n272));
  aobi12aa1n02x5               g177(.a(new_n269), .b(new_n268), .c(new_n266), .out0(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  nano22aa1n02x4               g179(.a(new_n273), .b(new_n272), .c(new_n274), .out0(new_n275));
  aobi12aa1n02x5               g180(.a(new_n265), .b(new_n183), .c(new_n188), .out0(new_n276));
  aoai13aa1n02x5               g181(.a(new_n245), .b(new_n235), .c(new_n219), .d(new_n231), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n264), .clkout(new_n278));
  aoai13aa1n02x5               g183(.a(new_n267), .b(new_n278), .c(new_n277), .d(new_n252), .o1(new_n279));
  oai012aa1n02x5               g184(.a(new_n269), .b(new_n279), .c(new_n276), .o1(new_n280));
  aoi012aa1n02x5               g185(.a(new_n274), .b(new_n280), .c(new_n272), .o1(new_n281));
  norp02aa1n02x5               g186(.a(new_n281), .b(new_n275), .o1(\s[28] ));
  norb02aa1n02x5               g187(.a(new_n269), .b(new_n274), .out0(new_n283));
  aobi12aa1n02x5               g188(.a(new_n283), .b(new_n268), .c(new_n266), .out0(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n272), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  nano22aa1n02x4               g191(.a(new_n284), .b(new_n285), .c(new_n286), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n283), .b(new_n279), .c(new_n276), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n286), .b(new_n288), .c(new_n285), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n289), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n269), .b(new_n286), .c(new_n274), .out0(new_n292));
  aobi12aa1n02x5               g197(.a(new_n292), .b(new_n268), .c(new_n266), .out0(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n293), .b(new_n294), .c(new_n295), .out0(new_n296));
  oai012aa1n02x5               g201(.a(new_n292), .b(new_n279), .c(new_n276), .o1(new_n297));
  aoi012aa1n02x5               g202(.a(new_n295), .b(new_n297), .c(new_n294), .o1(new_n298));
  norp02aa1n02x5               g203(.a(new_n298), .b(new_n296), .o1(\s[30] ));
  norb02aa1n02x5               g204(.a(new_n292), .b(new_n295), .out0(new_n300));
  aobi12aa1n02x5               g205(.a(new_n300), .b(new_n268), .c(new_n266), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n300), .b(new_n279), .c(new_n276), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnbna2aa1n03x5               g212(.a(new_n108), .b(new_n102), .c(new_n103), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n108), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nona22aa1n02x4               g216(.a(new_n157), .b(new_n117), .c(new_n99), .out0(new_n312));
  xobna2aa1n03x5               g217(.a(new_n116), .b(new_n312), .c(new_n121), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g218(.a(new_n122), .b(new_n110), .c(new_n158), .o(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g220(.a(new_n113), .b(new_n314), .c(new_n114), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g222(.a(new_n125), .b(new_n159), .c(new_n160), .out0(\s[9] ));
endmodule


