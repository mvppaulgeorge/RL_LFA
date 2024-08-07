// Benchmark "adder" written by ABC on Thu Jul 11 11:50:34 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n324, new_n325;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(new_n100), .clkout(new_n101));
  and002aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\a[3] ), .clkout(new_n103));
  160nm_ficinv00aa1n08x5       g008(.clk(\b[2] ), .clkout(new_n104));
  nanp02aa1n02x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  aoi012aa1n02x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(\a[4] ), .clkout(new_n112));
  aboi22aa1n03x5               g017(.a(\b[3] ), .b(new_n112), .c(new_n103), .d(new_n104), .out0(new_n113));
  oaoi13aa1n02x5               g018(.a(new_n102), .b(new_n113), .c(new_n111), .d(new_n107), .o1(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .out0(new_n116));
  norp02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  norp02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nona23aa1n02x4               g025(.a(new_n120), .b(new_n118), .c(new_n117), .d(new_n119), .out0(new_n121));
  norp03aa1n02x5               g026(.a(new_n121), .b(new_n116), .c(new_n115), .o1(new_n122));
  oai012aa1n02x5               g027(.a(new_n118), .b(new_n119), .c(new_n117), .o1(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(\a[8] ), .clkout(new_n124));
  160nm_ficinv00aa1n08x5       g029(.clk(\b[7] ), .clkout(new_n125));
  norp02aa1n02x5               g030(.a(\b[6] ), .b(\a[7] ), .o1(new_n126));
  oaoi03aa1n02x5               g031(.a(new_n124), .b(new_n125), .c(new_n126), .o1(new_n127));
  oai013aa1n02x4               g032(.a(new_n127), .b(new_n116), .c(new_n115), .d(new_n123), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n128), .c(new_n114), .d(new_n122), .o1(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n99), .b(new_n130), .c(new_n101), .out0(\s[10] ));
  aoi012aa1n02x5               g036(.a(new_n128), .b(new_n114), .c(new_n122), .o1(new_n132));
  oai012aa1n02x5               g037(.a(new_n98), .b(new_n100), .c(new_n97), .o1(new_n133));
  nano23aa1n02x4               g038(.a(new_n97), .b(new_n100), .c(new_n129), .d(new_n98), .out0(new_n134));
  oaib12aa1n02x5               g039(.a(new_n133), .b(new_n132), .c(new_n134), .out0(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g041(.clk(\a[12] ), .clkout(new_n137));
  norp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  xorc02aa1n02x5               g043(.a(\a[11] ), .b(\b[10] ), .out0(new_n139));
  aoi012aa1n02x5               g044(.a(new_n138), .b(new_n135), .c(new_n139), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(new_n137), .out0(\s[12] ));
  xorc02aa1n02x5               g046(.a(\a[12] ), .b(\b[11] ), .out0(new_n142));
  and003aa1n02x5               g047(.a(new_n134), .b(new_n139), .c(new_n142), .o(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n128), .c(new_n114), .d(new_n122), .o1(new_n144));
  xnrc02aa1n02x5               g049(.a(\b[10] ), .b(\a[11] ), .out0(new_n145));
  xnrc02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .out0(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(\b[11] ), .clkout(new_n147));
  oaoi03aa1n02x5               g052(.a(new_n137), .b(new_n147), .c(new_n138), .o1(new_n148));
  oai013aa1n02x4               g053(.a(new_n148), .b(new_n146), .c(new_n145), .d(new_n133), .o1(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  nanp02aa1n02x5               g055(.a(new_n144), .b(new_n150), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nona23aa1n02x4               g063(.a(new_n158), .b(new_n154), .c(new_n153), .d(new_n157), .out0(new_n159));
  oai012aa1n02x5               g064(.a(new_n158), .b(new_n157), .c(new_n153), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n144), .d(new_n150), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  160nm_ficinv00aa1n08x5       g070(.clk(new_n165), .clkout(new_n166));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(new_n169), .clkout(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n163), .c(new_n161), .d(new_n166), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nano23aa1n02x4               g078(.a(new_n153), .b(new_n157), .c(new_n158), .d(new_n154), .out0(new_n174));
  nona22aa1n02x4               g079(.a(new_n174), .b(new_n169), .c(new_n165), .out0(new_n175));
  nano32aa1n02x4               g080(.a(new_n175), .b(new_n134), .c(new_n142), .d(new_n139), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n128), .c(new_n114), .d(new_n122), .o1(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(new_n167), .clkout(new_n178));
  nona22aa1n02x4               g083(.a(new_n166), .b(new_n160), .c(new_n169), .out0(new_n179));
  aoi112aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(new_n180), .clkout(new_n181));
  oaoi03aa1n02x5               g086(.a(\a[10] ), .b(\b[9] ), .c(new_n101), .o1(new_n182));
  nanp03aa1n02x5               g087(.a(new_n182), .b(new_n139), .c(new_n142), .o1(new_n183));
  aoi012aa1n02x5               g088(.a(new_n175), .b(new_n183), .c(new_n148), .o1(new_n184));
  nano32aa1n02x4               g089(.a(new_n184), .b(new_n181), .c(new_n179), .d(new_n178), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n185), .b(new_n177), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[18] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[17] ), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(\b[16] ), .clkout(new_n190));
  oaoi03aa1n02x5               g095(.a(new_n189), .b(new_n190), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  xroi22aa1d04x5               g097(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n193));
  nanp02aa1n02x5               g098(.a(new_n190), .b(new_n189), .o1(new_n194));
  oaoi03aa1n02x5               g099(.a(\a[18] ), .b(\b[17] ), .c(new_n194), .o1(new_n195));
  norp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n195), .c(new_n186), .d(new_n193), .o1(new_n199));
  aoi112aa1n02x5               g104(.a(new_n198), .b(new_n195), .c(new_n186), .d(new_n193), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  nona22aa1n02x4               g110(.a(new_n199), .b(new_n205), .c(new_n196), .out0(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n196), .clkout(new_n207));
  aobi12aa1n02x5               g112(.a(new_n205), .b(new_n199), .c(new_n207), .out0(new_n208));
  norb02aa1n02x5               g113(.a(new_n206), .b(new_n208), .out0(\s[20] ));
  nano23aa1n02x4               g114(.a(new_n196), .b(new_n203), .c(new_n204), .d(new_n197), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n193), .b(new_n210), .o1(new_n211));
  oai022aa1n02x5               g116(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n212));
  oaib12aa1n02x5               g117(.a(new_n212), .b(new_n188), .c(\b[17] ), .out0(new_n213));
  nona23aa1n02x4               g118(.a(new_n204), .b(new_n197), .c(new_n196), .d(new_n203), .out0(new_n214));
  aoi012aa1n02x5               g119(.a(new_n203), .b(new_n196), .c(new_n204), .o1(new_n215));
  oai012aa1n02x5               g120(.a(new_n215), .b(new_n214), .c(new_n213), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n211), .c(new_n185), .d(new_n177), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xorc02aa1n02x5               g125(.a(\a[21] ), .b(\b[20] ), .out0(new_n221));
  xorc02aa1n02x5               g126(.a(\a[22] ), .b(\b[21] ), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(new_n220), .b(new_n222), .c(new_n218), .d(new_n221), .o1(new_n223));
  aoai13aa1n02x5               g128(.a(new_n222), .b(new_n220), .c(new_n218), .d(new_n221), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g130(.clk(\a[21] ), .clkout(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\a[22] ), .clkout(new_n227));
  xroi22aa1d04x5               g132(.a(new_n226), .b(\b[20] ), .c(new_n227), .d(\b[21] ), .out0(new_n228));
  nanp03aa1n02x5               g133(.a(new_n228), .b(new_n193), .c(new_n210), .o1(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\b[21] ), .clkout(new_n230));
  oao003aa1n02x5               g135(.a(new_n227), .b(new_n230), .c(new_n220), .carry(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n216), .c(new_n228), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n229), .c(new_n185), .d(new_n177), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  xorc02aa1n02x5               g141(.a(\a[24] ), .b(\b[23] ), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n237), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g145(.clk(\a[23] ), .clkout(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(\a[24] ), .clkout(new_n242));
  xroi22aa1d04x5               g147(.a(new_n241), .b(\b[22] ), .c(new_n242), .d(\b[23] ), .out0(new_n243));
  nano22aa1n02x4               g148(.a(new_n211), .b(new_n228), .c(new_n243), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n215), .clkout(new_n245));
  aoai13aa1n02x5               g150(.a(new_n228), .b(new_n245), .c(new_n210), .d(new_n195), .o1(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n231), .clkout(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n243), .clkout(new_n248));
  aoi112aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n249));
  aoib12aa1n02x5               g154(.a(new_n249), .b(new_n242), .c(\b[23] ), .out0(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n248), .c(new_n246), .d(new_n247), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n251), .c(new_n186), .d(new_n244), .o1(new_n253));
  aoi112aa1n02x5               g158(.a(new_n252), .b(new_n251), .c(new_n186), .d(new_n244), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n253), .b(new_n254), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  nona22aa1n02x4               g162(.a(new_n253), .b(new_n257), .c(new_n256), .out0(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n256), .clkout(new_n259));
  aobi12aa1n02x5               g164(.a(new_n257), .b(new_n253), .c(new_n259), .out0(new_n260));
  norb02aa1n02x5               g165(.a(new_n258), .b(new_n260), .out0(\s[26] ));
  oai012aa1n02x5               g166(.a(new_n113), .b(new_n111), .c(new_n107), .o1(new_n262));
  oaib12aa1n02x5               g167(.a(new_n262), .b(new_n112), .c(\b[3] ), .out0(new_n263));
  orn003aa1n02x5               g168(.a(new_n121), .b(new_n115), .c(new_n116), .o(new_n264));
  oabi12aa1n02x5               g169(.a(new_n128), .b(new_n264), .c(new_n263), .out0(new_n265));
  norp03aa1n02x5               g170(.a(new_n160), .b(new_n169), .c(new_n165), .o1(new_n266));
  nona32aa1n02x4               g171(.a(new_n149), .b(new_n169), .c(new_n159), .d(new_n165), .out0(new_n267));
  nona32aa1n02x4               g172(.a(new_n267), .b(new_n180), .c(new_n266), .d(new_n167), .out0(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(\a[25] ), .clkout(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(\a[26] ), .clkout(new_n270));
  xroi22aa1d04x5               g175(.a(new_n269), .b(\b[24] ), .c(new_n270), .d(\b[25] ), .out0(new_n271));
  nano32aa1n02x4               g176(.a(new_n211), .b(new_n271), .c(new_n228), .d(new_n243), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n268), .c(new_n265), .d(new_n176), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n259), .carry(new_n274));
  aobi12aa1n02x5               g179(.a(new_n274), .b(new_n251), .c(new_n271), .out0(new_n275));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  norb02aa1n02x5               g182(.a(new_n277), .b(new_n276), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n275), .c(new_n273), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n276), .clkout(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  aobi12aa1n02x5               g186(.a(new_n272), .b(new_n185), .c(new_n177), .out0(new_n282));
  aoai13aa1n02x5               g187(.a(new_n243), .b(new_n231), .c(new_n216), .d(new_n228), .o1(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n271), .clkout(new_n284));
  aoai13aa1n02x5               g189(.a(new_n274), .b(new_n284), .c(new_n283), .d(new_n250), .o1(new_n285));
  oai012aa1n02x5               g190(.a(new_n277), .b(new_n285), .c(new_n282), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n281), .b(new_n286), .c(new_n280), .o1(new_n287));
  aobi12aa1n02x5               g192(.a(new_n277), .b(new_n275), .c(new_n273), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n288), .b(new_n280), .c(new_n281), .out0(new_n289));
  norp02aa1n02x5               g194(.a(new_n287), .b(new_n289), .o1(\s[28] ));
  nano22aa1n02x4               g195(.a(new_n281), .b(new_n280), .c(new_n277), .out0(new_n291));
  oai012aa1n02x5               g196(.a(new_n291), .b(new_n285), .c(new_n282), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n280), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n02x5               g200(.a(new_n291), .b(new_n275), .c(new_n273), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n295), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g204(.a(new_n278), .b(new_n294), .c(new_n281), .out0(new_n300));
  oai012aa1n02x5               g205(.a(new_n300), .b(new_n285), .c(new_n282), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  aoi012aa1n02x5               g208(.a(new_n303), .b(new_n301), .c(new_n302), .o1(new_n304));
  aobi12aa1n02x5               g209(.a(new_n300), .b(new_n275), .c(new_n273), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n305), .b(new_n302), .c(new_n303), .out0(new_n306));
  norp02aa1n02x5               g211(.a(new_n304), .b(new_n306), .o1(\s[30] ));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  norb03aa1n02x5               g213(.a(new_n291), .b(new_n303), .c(new_n294), .out0(new_n309));
  aobi12aa1n02x5               g214(.a(new_n309), .b(new_n275), .c(new_n273), .out0(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n311));
  nano22aa1n02x4               g216(.a(new_n310), .b(new_n308), .c(new_n311), .out0(new_n312));
  oai012aa1n02x5               g217(.a(new_n309), .b(new_n285), .c(new_n282), .o1(new_n313));
  aoi012aa1n02x5               g218(.a(new_n308), .b(new_n313), .c(new_n311), .o1(new_n314));
  norp02aa1n02x5               g219(.a(new_n314), .b(new_n312), .o1(\s[31] ));
  xnbna2aa1n03x5               g220(.a(new_n111), .b(new_n105), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n114), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g224(.a(new_n120), .b(new_n114), .c(new_n119), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nona22aa1n02x4               g226(.a(new_n262), .b(new_n121), .c(new_n102), .out0(new_n322));
  xobna2aa1n03x5               g227(.a(new_n116), .b(new_n322), .c(new_n123), .out0(\s[7] ));
  orn002aa1n02x5               g228(.a(\a[7] ), .b(\b[6] ), .o(new_n324));
  aoai13aa1n02x5               g229(.a(new_n324), .b(new_n116), .c(new_n322), .d(new_n123), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g231(.a(new_n132), .b(new_n129), .c(new_n101), .out0(\s[9] ));
endmodule


