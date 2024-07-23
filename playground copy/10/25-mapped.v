// Benchmark "adder" written by ABC on Thu Jul 11 11:38:37 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n318, new_n319, new_n320,
    new_n322, new_n323, new_n324;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nona23aa1n02x4               g008(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .out0(new_n105));
  xnrc02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .out0(new_n106));
  norp03aa1n02x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  nanp02aa1n02x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[0] ), .b(\a[1] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n113), .b(new_n115), .c(new_n114), .o1(new_n116));
  160nm_fiao0012aa1n02p5x5     g021(.a(new_n108), .b(new_n110), .c(new_n109), .o(new_n117));
  oabi12aa1n02x5               g022(.a(new_n117), .b(new_n112), .c(new_n116), .out0(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(new_n100), .clkout(new_n119));
  nanp02aa1n02x5               g024(.a(new_n102), .b(new_n101), .o1(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[5] ), .clkout(new_n121));
  oai022aa1n02x5               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  oaib12aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(\a[6] ), .out0(new_n123));
  oai112aa1n02x5               g028(.a(new_n119), .b(new_n120), .c(new_n104), .d(new_n123), .o1(new_n124));
  norp02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n124), .c(new_n118), .d(new_n107), .o1(new_n128));
  oaoi13aa1n02x5               g033(.a(new_n99), .b(new_n128), .c(\a[9] ), .d(\b[8] ), .o1(new_n129));
  160nm_ficinv00aa1n08x5       g034(.clk(new_n98), .clkout(new_n130));
  nona32aa1n02x4               g035(.a(new_n128), .b(new_n125), .c(new_n130), .d(new_n97), .out0(new_n131));
  nanb02aa1n02x5               g036(.a(new_n129), .b(new_n131), .out0(\s[10] ));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n131), .c(new_n98), .out0(\s[11] ));
  aoi013aa1n02x4               g041(.a(new_n133), .b(new_n131), .c(new_n98), .d(new_n134), .o1(new_n137));
  norp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n138), .clkout(new_n139));
  nanp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n137), .b(new_n140), .c(new_n139), .out0(\s[12] ));
  norb02aa1n02x5               g046(.a(new_n134), .b(new_n133), .out0(new_n142));
  nanb02aa1n02x5               g047(.a(new_n138), .b(new_n140), .out0(new_n143));
  nano32aa1n02x4               g048(.a(new_n143), .b(new_n142), .c(new_n127), .d(new_n99), .out0(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n124), .c(new_n118), .d(new_n107), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(new_n133), .b(new_n140), .o1(new_n146));
  oai022aa1n02x5               g051(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n147));
  nano23aa1n02x4               g052(.a(new_n143), .b(new_n135), .c(new_n147), .d(new_n98), .out0(new_n148));
  nano22aa1n02x4               g053(.a(new_n148), .b(new_n139), .c(new_n146), .out0(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(\a[13] ), .clkout(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(\b[12] ), .clkout(new_n151));
  nanp02aa1n02x5               g056(.a(new_n151), .b(new_n150), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n152), .b(new_n153), .o1(new_n154));
  xobna2aa1n03x5               g059(.a(new_n154), .b(new_n145), .c(new_n149), .out0(\s[13] ));
  aoai13aa1n02x5               g060(.a(new_n152), .b(new_n154), .c(new_n145), .d(new_n149), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  norp02aa1n02x5               g065(.a(new_n160), .b(new_n154), .o1(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  aoai13aa1n02x5               g067(.a(new_n159), .b(new_n158), .c(new_n150), .d(new_n151), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n162), .c(new_n145), .d(new_n149), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  norb02aa1n02x5               g078(.a(new_n140), .b(new_n138), .out0(new_n174));
  nano23aa1n02x4               g079(.a(new_n97), .b(new_n125), .c(new_n126), .d(new_n98), .out0(new_n175));
  nano23aa1n02x4               g080(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n176));
  nona22aa1n02x4               g081(.a(new_n176), .b(new_n160), .c(new_n154), .out0(new_n177));
  nano32aa1n02x4               g082(.a(new_n177), .b(new_n175), .c(new_n174), .d(new_n142), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n124), .c(new_n118), .d(new_n107), .o1(new_n179));
  norp02aa1n02x5               g084(.a(new_n125), .b(new_n97), .o1(new_n180));
  nona23aa1n02x4               g085(.a(new_n174), .b(new_n142), .c(new_n180), .d(new_n130), .out0(new_n181));
  nanp03aa1n02x5               g086(.a(new_n181), .b(new_n139), .c(new_n146), .o1(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n177), .clkout(new_n183));
  nona23aa1n02x4               g088(.a(new_n169), .b(new_n167), .c(new_n166), .d(new_n168), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n166), .b(new_n169), .o1(new_n185));
  oai122aa1n02x7               g090(.a(new_n185), .b(new_n184), .c(new_n163), .d(\b[15] ), .e(\a[16] ), .o1(new_n186));
  aoi012aa1n02x5               g091(.a(new_n186), .b(new_n182), .c(new_n183), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(new_n187), .b(new_n179), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[18] ), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\a[17] ), .clkout(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(\b[16] ), .clkout(new_n192));
  oaoi03aa1n02x5               g097(.a(new_n191), .b(new_n192), .c(new_n188), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[17] ), .c(new_n190), .out0(\s[18] ));
  xroi22aa1d04x5               g099(.a(new_n191), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n195));
  nanp02aa1n02x5               g100(.a(new_n192), .b(new_n191), .o1(new_n196));
  oaoi03aa1n02x5               g101(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n197));
  norp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  aoai13aa1n02x5               g105(.a(new_n200), .b(new_n197), .c(new_n188), .d(new_n195), .o1(new_n201));
  aoi112aa1n02x5               g106(.a(new_n200), .b(new_n197), .c(new_n188), .d(new_n195), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  nona22aa1n02x4               g112(.a(new_n201), .b(new_n207), .c(new_n198), .out0(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n207), .clkout(new_n209));
  oaoi13aa1n02x5               g114(.a(new_n209), .b(new_n201), .c(\a[19] ), .d(\b[18] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n208), .b(new_n210), .out0(\s[20] ));
  nano23aa1n02x4               g116(.a(new_n198), .b(new_n205), .c(new_n206), .d(new_n199), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n195), .b(new_n212), .o1(new_n213));
  oai022aa1n02x5               g118(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n214));
  oaib12aa1n02x5               g119(.a(new_n214), .b(new_n190), .c(\b[17] ), .out0(new_n215));
  nona23aa1n02x4               g120(.a(new_n206), .b(new_n199), .c(new_n198), .d(new_n205), .out0(new_n216));
  aoi012aa1n02x5               g121(.a(new_n205), .b(new_n198), .c(new_n206), .o1(new_n217));
  oai012aa1n02x5               g122(.a(new_n217), .b(new_n216), .c(new_n215), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n213), .c(new_n187), .d(new_n179), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[21] ), .b(\b[20] ), .out0(new_n223));
  xorc02aa1n02x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n222), .b(new_n224), .c(new_n220), .d(new_n223), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n224), .b(new_n222), .c(new_n220), .d(new_n223), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g132(.clk(\a[21] ), .clkout(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(\a[22] ), .clkout(new_n229));
  xroi22aa1d04x5               g134(.a(new_n228), .b(\b[20] ), .c(new_n229), .d(\b[21] ), .out0(new_n230));
  nanp03aa1n02x5               g135(.a(new_n230), .b(new_n195), .c(new_n212), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(\b[21] ), .clkout(new_n232));
  oaoi03aa1n02x5               g137(.a(new_n229), .b(new_n232), .c(new_n222), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n218), .c(new_n230), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n231), .c(new_n187), .d(new_n179), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  xorc02aa1n02x5               g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  xorc02aa1n02x5               g144(.a(\a[24] ), .b(\b[23] ), .out0(new_n240));
  aoi112aa1n02x5               g145(.a(new_n238), .b(new_n240), .c(new_n236), .d(new_n239), .o1(new_n241));
  aoai13aa1n02x5               g146(.a(new_n240), .b(new_n238), .c(new_n236), .d(new_n239), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(\s[24] ));
  and002aa1n02x5               g148(.a(new_n240), .b(new_n239), .o(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  nano32aa1n02x4               g150(.a(new_n245), .b(new_n230), .c(new_n195), .d(new_n212), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n217), .clkout(new_n247));
  aoai13aa1n02x5               g152(.a(new_n230), .b(new_n247), .c(new_n212), .d(new_n197), .o1(new_n248));
  orn002aa1n02x5               g153(.a(\a[23] ), .b(\b[22] ), .o(new_n249));
  oao003aa1n02x5               g154(.a(\a[24] ), .b(\b[23] ), .c(new_n249), .carry(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n245), .c(new_n248), .d(new_n233), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n251), .c(new_n188), .d(new_n246), .o1(new_n253));
  aoi112aa1n02x5               g158(.a(new_n252), .b(new_n251), .c(new_n188), .d(new_n246), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n253), .b(new_n254), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  nona22aa1n02x4               g162(.a(new_n253), .b(new_n257), .c(new_n256), .out0(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n256), .clkout(new_n259));
  aobi12aa1n02x5               g164(.a(new_n257), .b(new_n253), .c(new_n259), .out0(new_n260));
  norb02aa1n02x5               g165(.a(new_n258), .b(new_n260), .out0(\s[26] ));
  nanp02aa1n02x5               g166(.a(new_n118), .b(new_n107), .o1(new_n262));
  nanb02aa1n02x5               g167(.a(new_n124), .b(new_n262), .out0(new_n263));
  oabi12aa1n02x5               g168(.a(new_n186), .b(new_n149), .c(new_n177), .out0(new_n264));
  and002aa1n02x5               g169(.a(new_n257), .b(new_n252), .o(new_n265));
  nano22aa1n02x4               g170(.a(new_n231), .b(new_n244), .c(new_n265), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n264), .c(new_n263), .d(new_n178), .o1(new_n267));
  oao003aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .c(new_n259), .carry(new_n268));
  aobi12aa1n02x5               g173(.a(new_n268), .b(new_n251), .c(new_n265), .out0(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n269), .c(new_n267), .out0(\s[27] ));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n272), .clkout(new_n273));
  aobi12aa1n02x5               g178(.a(new_n270), .b(new_n269), .c(new_n267), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  nano22aa1n02x4               g180(.a(new_n274), .b(new_n273), .c(new_n275), .out0(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n266), .clkout(new_n277));
  aoi012aa1n02x5               g182(.a(new_n277), .b(new_n187), .c(new_n179), .o1(new_n278));
  aoai13aa1n02x5               g183(.a(new_n244), .b(new_n234), .c(new_n218), .d(new_n230), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n265), .clkout(new_n280));
  aoai13aa1n02x5               g185(.a(new_n268), .b(new_n280), .c(new_n279), .d(new_n250), .o1(new_n281));
  oai012aa1n02x5               g186(.a(new_n270), .b(new_n281), .c(new_n278), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n275), .b(new_n282), .c(new_n273), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n283), .b(new_n276), .o1(\s[28] ));
  norb02aa1n02x5               g189(.a(new_n270), .b(new_n275), .out0(new_n285));
  oai012aa1n02x5               g190(.a(new_n285), .b(new_n281), .c(new_n278), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n273), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  aobi12aa1n02x5               g194(.a(new_n285), .b(new_n269), .c(new_n267), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n287), .c(new_n288), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n114), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g198(.a(new_n270), .b(new_n288), .c(new_n275), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n294), .b(new_n281), .c(new_n278), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x5               g203(.a(new_n294), .b(new_n269), .c(new_n267), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  norb02aa1n02x5               g206(.a(new_n294), .b(new_n297), .out0(new_n302));
  aobi12aa1n02x5               g207(.a(new_n302), .b(new_n269), .c(new_n267), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  oai012aa1n02x5               g211(.a(new_n302), .b(new_n281), .c(new_n278), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n116), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n116), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g218(.a(new_n106), .b(new_n118), .out0(new_n314));
  oai012aa1n02x5               g219(.a(new_n314), .b(\b[4] ), .c(\a[5] ), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g221(.a(new_n102), .b(new_n103), .out0(new_n317));
  nanb02aa1n02x5               g222(.a(new_n105), .b(new_n315), .out0(new_n318));
  oaoi13aa1n02x5               g223(.a(new_n317), .b(new_n318), .c(\a[6] ), .d(\b[5] ), .o1(new_n319));
  oai112aa1n02x5               g224(.a(new_n318), .b(new_n317), .c(\b[5] ), .d(\a[6] ), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n320), .b(new_n319), .out0(\s[7] ));
  nanb02aa1n02x5               g226(.a(new_n100), .b(new_n101), .out0(new_n322));
  oab012aa1n02x4               g227(.a(new_n322), .b(new_n319), .c(new_n102), .out0(new_n323));
  aoi112aa1n02x5               g228(.a(new_n319), .b(new_n102), .c(new_n119), .d(new_n101), .o1(new_n324));
  norp02aa1n02x5               g229(.a(new_n323), .b(new_n324), .o1(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n263), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


