// Benchmark "adder" written by ABC on Thu Jul 11 12:21:10 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n323, new_n325;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n02x4               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  norp03aa1n02x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[3] ), .b(\a[4] ), .out0(new_n110));
  norp02aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanb02aa1n02x5               g017(.a(new_n111), .b(new_n112), .out0(new_n113));
  oai022aa1n02x5               g018(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n114));
  aob012aa1n02x5               g019(.a(new_n114), .b(\b[3] ), .c(\a[4] ), .out0(new_n115));
  oai013aa1n02x4               g020(.a(new_n115), .b(new_n110), .c(new_n109), .d(new_n113), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(new_n116), .b(new_n105), .o1(new_n117));
  nano23aa1n02x4               g022(.a(new_n98), .b(new_n100), .c(new_n101), .d(new_n99), .out0(new_n118));
  aoi112aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\a[5] ), .clkout(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[4] ), .clkout(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n120), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n122), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(new_n119), .b(new_n98), .c(new_n118), .d(new_n123), .o1(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n97), .b(new_n125), .c(new_n117), .d(new_n124), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xnrc02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .out0(new_n128));
  160nm_ficinv00aa1n08x5       g033(.clk(new_n128), .clkout(new_n129));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  oaoi03aa1n02x5               g037(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n132), .b(new_n133), .c(new_n126), .d(new_n129), .o1(new_n134));
  aoi112aa1n02x5               g039(.a(new_n132), .b(new_n133), .c(new_n126), .d(new_n129), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(\s[11] ));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  nona22aa1n02x4               g044(.a(new_n134), .b(new_n139), .c(new_n130), .out0(new_n140));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n139), .clkout(new_n141));
  oaoi13aa1n02x5               g046(.a(new_n141), .b(new_n134), .c(\a[11] ), .d(\b[10] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n140), .b(new_n142), .out0(\s[12] ));
  nano23aa1n02x4               g048(.a(new_n130), .b(new_n137), .c(new_n138), .d(new_n131), .out0(new_n144));
  nona22aa1n02x4               g049(.a(new_n144), .b(new_n125), .c(new_n128), .out0(new_n145));
  oai022aa1n02x5               g050(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n146));
  aob012aa1n02x5               g051(.a(new_n146), .b(\b[9] ), .c(\a[10] ), .out0(new_n147));
  nona23aa1n02x4               g052(.a(new_n138), .b(new_n131), .c(new_n130), .d(new_n137), .out0(new_n148));
  160nm_fiao0012aa1n02p5x5     g053(.a(new_n137), .b(new_n130), .c(new_n138), .o(new_n149));
  oabi12aa1n02x5               g054(.a(new_n149), .b(new_n148), .c(new_n147), .out0(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n150), .clkout(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n145), .c(new_n117), .d(new_n124), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g058(.clk(\a[14] ), .clkout(new_n154));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  xnrc02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .out0(new_n156));
  aoib12aa1n02x5               g061(.a(new_n155), .b(new_n152), .c(new_n156), .out0(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(new_n154), .out0(\s[14] ));
  xnrc02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .out0(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(\b[13] ), .clkout(new_n161));
  oaoi03aa1n02x5               g066(.a(new_n154), .b(new_n161), .c(new_n155), .o1(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  xnrc02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .out0(new_n164));
  norp02aa1n02x5               g069(.a(new_n164), .b(new_n156), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n160), .b(new_n163), .c(new_n152), .d(new_n165), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n163), .b(new_n160), .c(new_n152), .d(new_n165), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[15] ));
  xnrc02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .out0(new_n169));
  oai112aa1n02x5               g074(.a(new_n166), .b(new_n169), .c(\b[14] ), .d(\a[15] ), .o1(new_n170));
  oaoi13aa1n02x5               g075(.a(new_n169), .b(new_n166), .c(\a[15] ), .d(\b[14] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(\s[16] ));
  160nm_ficinv00aa1n08x5       g077(.clk(new_n119), .clkout(new_n173));
  oai022aa1n02x5               g078(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n174));
  aob012aa1n02x5               g079(.a(new_n174), .b(\b[5] ), .c(\a[6] ), .out0(new_n175));
  oai122aa1n02x7               g080(.a(new_n173), .b(new_n102), .c(new_n175), .d(\b[7] ), .e(\a[8] ), .o1(new_n176));
  norp02aa1n02x5               g081(.a(new_n169), .b(new_n159), .o1(new_n177));
  nano22aa1n02x4               g082(.a(new_n145), .b(new_n165), .c(new_n177), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n176), .c(new_n105), .d(new_n116), .o1(new_n179));
  aoai13aa1n02x5               g084(.a(new_n177), .b(new_n163), .c(new_n150), .d(new_n165), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n181));
  oab012aa1n02x4               g086(.a(new_n181), .b(\a[16] ), .c(\b[15] ), .out0(new_n182));
  nanp03aa1n02x5               g087(.a(new_n179), .b(new_n180), .c(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  norp03aa1n02x5               g094(.a(new_n148), .b(new_n125), .c(new_n128), .o1(new_n190));
  nanp03aa1n02x5               g095(.a(new_n190), .b(new_n165), .c(new_n177), .o1(new_n191));
  aoi012aa1n02x5               g096(.a(new_n191), .b(new_n117), .c(new_n124), .o1(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n177), .clkout(new_n193));
  aoai13aa1n02x5               g098(.a(new_n165), .b(new_n149), .c(new_n144), .d(new_n133), .o1(new_n194));
  aoai13aa1n02x5               g099(.a(new_n182), .b(new_n193), .c(new_n194), .d(new_n162), .o1(new_n195));
  xroi22aa1d04x5               g100(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n196));
  oai012aa1n02x5               g101(.a(new_n196), .b(new_n195), .c(new_n192), .o1(new_n197));
  oai022aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n198));
  oaib12aa1n02x5               g103(.a(new_n198), .b(new_n185), .c(\b[17] ), .out0(new_n199));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n202), .clkout(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n197), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g110(.clk(new_n200), .clkout(new_n206));
  aoi012aa1n02x5               g111(.a(new_n202), .b(new_n197), .c(new_n199), .o1(new_n207));
  norp02aa1n02x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  nano22aa1n02x4               g115(.a(new_n207), .b(new_n206), .c(new_n210), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n187), .b(new_n186), .o1(new_n212));
  oaoi03aa1n02x5               g117(.a(\a[18] ), .b(\b[17] ), .c(new_n212), .o1(new_n213));
  aoai13aa1n02x5               g118(.a(new_n203), .b(new_n213), .c(new_n183), .d(new_n196), .o1(new_n214));
  aoi012aa1n02x5               g119(.a(new_n210), .b(new_n214), .c(new_n206), .o1(new_n215));
  norp02aa1n02x5               g120(.a(new_n215), .b(new_n211), .o1(\s[20] ));
  nano23aa1n02x4               g121(.a(new_n200), .b(new_n208), .c(new_n209), .d(new_n201), .out0(new_n217));
  nanp02aa1n02x5               g122(.a(new_n196), .b(new_n217), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n195), .c(new_n192), .o1(new_n220));
  nona23aa1n02x4               g125(.a(new_n209), .b(new_n201), .c(new_n200), .d(new_n208), .out0(new_n221));
  aoi012aa1n02x5               g126(.a(new_n208), .b(new_n200), .c(new_n209), .o1(new_n222));
  oai012aa1n02x5               g127(.a(new_n222), .b(new_n221), .c(new_n199), .o1(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n223), .clkout(new_n224));
  norp02aa1n02x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n220), .c(new_n224), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n225), .clkout(new_n229));
  aobi12aa1n02x5               g134(.a(new_n227), .b(new_n220), .c(new_n224), .out0(new_n230));
  xnrc02aa1n02x5               g135(.a(\b[21] ), .b(\a[22] ), .out0(new_n231));
  nano22aa1n02x4               g136(.a(new_n230), .b(new_n229), .c(new_n231), .out0(new_n232));
  aoai13aa1n02x5               g137(.a(new_n227), .b(new_n223), .c(new_n183), .d(new_n219), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n231), .b(new_n233), .c(new_n229), .o1(new_n234));
  norp02aa1n02x5               g139(.a(new_n234), .b(new_n232), .o1(\s[22] ));
  nano22aa1n02x4               g140(.a(new_n231), .b(new_n229), .c(new_n226), .out0(new_n236));
  and003aa1n02x5               g141(.a(new_n196), .b(new_n236), .c(new_n217), .o(new_n237));
  oai012aa1n02x5               g142(.a(new_n237), .b(new_n195), .c(new_n192), .o1(new_n238));
  oao003aa1n02x5               g143(.a(\a[22] ), .b(\b[21] ), .c(new_n229), .carry(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  aoi012aa1n02x5               g145(.a(new_n240), .b(new_n223), .c(new_n236), .o1(new_n241));
  xnrc02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  xnbna2aa1n03x5               g148(.a(new_n243), .b(new_n238), .c(new_n241), .out0(\s[23] ));
  norp02aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n245), .clkout(new_n246));
  aoi012aa1n02x5               g151(.a(new_n242), .b(new_n238), .c(new_n241), .o1(new_n247));
  xnrc02aa1n02x5               g152(.a(\b[23] ), .b(\a[24] ), .out0(new_n248));
  nano22aa1n02x4               g153(.a(new_n247), .b(new_n246), .c(new_n248), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n241), .clkout(new_n250));
  aoai13aa1n02x5               g155(.a(new_n243), .b(new_n250), .c(new_n183), .d(new_n237), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n248), .b(new_n251), .c(new_n246), .o1(new_n252));
  norp02aa1n02x5               g157(.a(new_n252), .b(new_n249), .o1(\s[24] ));
  norp02aa1n02x5               g158(.a(new_n248), .b(new_n242), .o1(new_n254));
  nano22aa1n02x4               g159(.a(new_n218), .b(new_n236), .c(new_n254), .out0(new_n255));
  oai012aa1n02x5               g160(.a(new_n255), .b(new_n195), .c(new_n192), .o1(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n222), .clkout(new_n257));
  aoai13aa1n02x5               g162(.a(new_n236), .b(new_n257), .c(new_n217), .d(new_n213), .o1(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n254), .clkout(new_n259));
  oao003aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .carry(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n259), .c(new_n258), .d(new_n239), .o1(new_n261));
  xnrc02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .out0(new_n262));
  aoib12aa1n02x5               g167(.a(new_n262), .b(new_n256), .c(new_n261), .out0(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n262), .clkout(new_n264));
  aoi112aa1n02x5               g169(.a(new_n264), .b(new_n261), .c(new_n183), .d(new_n255), .o1(new_n265));
  norp02aa1n02x5               g170(.a(new_n263), .b(new_n265), .o1(\s[25] ));
  norp02aa1n02x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n267), .clkout(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[25] ), .b(\a[26] ), .out0(new_n269));
  nano22aa1n02x4               g174(.a(new_n263), .b(new_n268), .c(new_n269), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n264), .b(new_n261), .c(new_n183), .d(new_n255), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n269), .b(new_n271), .c(new_n268), .o1(new_n272));
  norp02aa1n02x5               g177(.a(new_n272), .b(new_n270), .o1(\s[26] ));
  norp02aa1n02x5               g178(.a(new_n269), .b(new_n262), .o1(new_n274));
  nano32aa1n02x4               g179(.a(new_n218), .b(new_n274), .c(new_n236), .d(new_n254), .out0(new_n275));
  oai012aa1n02x5               g180(.a(new_n275), .b(new_n195), .c(new_n192), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n277));
  aobi12aa1n02x5               g182(.a(new_n277), .b(new_n261), .c(new_n274), .out0(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n276), .c(new_n278), .out0(\s[27] ));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n281), .clkout(new_n282));
  aobi12aa1n02x5               g187(.a(new_n279), .b(new_n276), .c(new_n278), .out0(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n283), .b(new_n282), .c(new_n284), .out0(new_n285));
  aoai13aa1n02x5               g190(.a(new_n254), .b(new_n240), .c(new_n223), .d(new_n236), .o1(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n274), .clkout(new_n287));
  aoai13aa1n02x5               g192(.a(new_n277), .b(new_n287), .c(new_n286), .d(new_n260), .o1(new_n288));
  aoai13aa1n02x5               g193(.a(new_n279), .b(new_n288), .c(new_n183), .d(new_n275), .o1(new_n289));
  aoi012aa1n02x5               g194(.a(new_n284), .b(new_n289), .c(new_n282), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n290), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g196(.a(new_n279), .b(new_n284), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n288), .c(new_n183), .d(new_n275), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n02x5               g201(.a(new_n292), .b(new_n276), .c(new_n278), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n296), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g205(.a(new_n279), .b(new_n295), .c(new_n284), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n288), .c(new_n183), .d(new_n275), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  aoi012aa1n02x5               g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  aobi12aa1n02x5               g210(.a(new_n301), .b(new_n276), .c(new_n278), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n306), .b(new_n303), .c(new_n304), .out0(new_n307));
  norp02aa1n02x5               g212(.a(new_n305), .b(new_n307), .o1(\s[30] ));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  norb02aa1n02x5               g214(.a(new_n301), .b(new_n304), .out0(new_n310));
  aobi12aa1n02x5               g215(.a(new_n310), .b(new_n276), .c(new_n278), .out0(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n312));
  nano22aa1n02x4               g217(.a(new_n311), .b(new_n309), .c(new_n312), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n310), .b(new_n288), .c(new_n183), .d(new_n275), .o1(new_n314));
  aoi012aa1n02x5               g219(.a(new_n309), .b(new_n314), .c(new_n312), .o1(new_n315));
  norp02aa1n02x5               g220(.a(new_n315), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n109), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaib12aa1n02x5               g225(.a(new_n122), .b(new_n104), .c(new_n116), .out0(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoib12aa1n02x5               g227(.a(new_n123), .b(new_n321), .c(new_n103), .out0(new_n323));
  xnrb03aa1n02x5               g228(.a(new_n323), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g229(.a(\a[7] ), .b(\b[6] ), .c(new_n323), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g231(.a(new_n125), .b(new_n117), .c(new_n124), .out0(\s[9] ));
endmodule


