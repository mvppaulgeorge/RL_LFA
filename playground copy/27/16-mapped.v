// Benchmark "adder" written by ABC on Thu Jul 11 12:53:00 2024

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
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n320,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nona23aa1n02x4               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .out0(new_n105));
  norp03aa1n02x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  oai012aa1n02x5               g014(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .out0(new_n111));
  norp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nanb02aa1n02x5               g018(.a(new_n112), .b(new_n113), .out0(new_n114));
  oai022aa1n02x5               g019(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n115));
  aob012aa1n02x5               g020(.a(new_n115), .b(\b[3] ), .c(\a[4] ), .out0(new_n116));
  oai013aa1n02x4               g021(.a(new_n116), .b(new_n111), .c(new_n110), .d(new_n114), .o1(new_n117));
  aoi112aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(new_n118), .clkout(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  aob012aa1n02x5               g025(.a(new_n120), .b(\b[5] ), .c(\a[6] ), .out0(new_n121));
  oai122aa1n02x7               g026(.a(new_n119), .b(new_n103), .c(new_n121), .d(\b[7] ), .e(\a[8] ), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n02x5               g028(.a(new_n123), .b(new_n97), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n122), .c(new_n106), .d(new_n117), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g034(.a(new_n117), .b(new_n106), .o1(new_n130));
  nano23aa1n02x4               g035(.a(new_n99), .b(new_n101), .c(new_n102), .d(new_n100), .out0(new_n131));
  160nm_ficinv00aa1n08x5       g036(.clk(\a[5] ), .clkout(new_n132));
  160nm_ficinv00aa1n08x5       g037(.clk(\b[4] ), .clkout(new_n133));
  nanp02aa1n02x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  oaoi03aa1n02x5               g039(.a(\a[6] ), .b(\b[5] ), .c(new_n134), .o1(new_n135));
  aoi112aa1n02x5               g040(.a(new_n118), .b(new_n99), .c(new_n131), .d(new_n135), .o1(new_n136));
  oai012aa1n02x5               g041(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n137));
  nona23aa1n02x4               g042(.a(new_n127), .b(new_n123), .c(new_n97), .d(new_n126), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n137), .b(new_n138), .c(new_n130), .d(new_n136), .o1(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  aoi012aa1n02x5               g047(.a(new_n141), .b(new_n139), .c(new_n142), .o1(new_n143));
  xnrb03aa1n02x5               g048(.a(new_n143), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nona23aa1n02x4               g051(.a(new_n146), .b(new_n142), .c(new_n141), .d(new_n145), .out0(new_n147));
  160nm_fiao0012aa1n02p5x5     g052(.a(new_n145), .b(new_n141), .c(new_n146), .o(new_n148));
  oabi12aa1n02x5               g053(.a(new_n148), .b(new_n147), .c(new_n137), .out0(new_n149));
  aoi012aa1n02x5               g054(.a(new_n122), .b(new_n106), .c(new_n117), .o1(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n138), .clkout(new_n151));
  nano23aa1n02x4               g056(.a(new_n141), .b(new_n145), .c(new_n146), .d(new_n142), .out0(new_n152));
  nano22aa1n02x4               g057(.a(new_n150), .b(new_n151), .c(new_n152), .out0(new_n153));
  xnrc02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .out0(new_n154));
  oabi12aa1n02x5               g059(.a(new_n154), .b(new_n153), .c(new_n149), .out0(new_n155));
  norb03aa1n02x5               g060(.a(new_n154), .b(new_n153), .c(new_n149), .out0(new_n156));
  norb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(\s[13] ));
  orn002aa1n02x5               g062(.a(\a[13] ), .b(\b[12] ), .o(new_n158));
  xnrc02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .out0(new_n159));
  xobna2aa1n03x5               g064(.a(new_n159), .b(new_n155), .c(new_n158), .out0(\s[14] ));
  norp02aa1n02x5               g065(.a(new_n159), .b(new_n154), .o1(new_n161));
  nano32aa1n02x4               g066(.a(new_n150), .b(new_n161), .c(new_n151), .d(new_n152), .out0(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n137), .clkout(new_n163));
  aoai13aa1n02x5               g068(.a(new_n161), .b(new_n148), .c(new_n152), .d(new_n163), .o1(new_n164));
  oao003aa1n02x5               g069(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .carry(new_n165));
  nanp02aa1n02x5               g070(.a(new_n164), .b(new_n165), .o1(new_n166));
  norp02aa1n02x5               g071(.a(new_n162), .b(new_n166), .o1(new_n167));
  xnrc02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .out0(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n168), .clkout(new_n169));
  xnrc02aa1n02x5               g074(.a(new_n167), .b(new_n169), .out0(\s[15] ));
  oaoi03aa1n02x5               g075(.a(\a[15] ), .b(\b[14] ), .c(new_n167), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  nanp03aa1n02x5               g077(.a(new_n152), .b(new_n124), .c(new_n128), .o1(new_n173));
  xnrc02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .out0(new_n174));
  norp02aa1n02x5               g079(.a(new_n174), .b(new_n168), .o1(new_n175));
  nano22aa1n02x4               g080(.a(new_n173), .b(new_n161), .c(new_n175), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n122), .c(new_n106), .d(new_n117), .o1(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(new_n165), .clkout(new_n178));
  aoai13aa1n02x5               g083(.a(new_n175), .b(new_n178), .c(new_n149), .d(new_n161), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n180));
  oab012aa1n02x4               g085(.a(new_n180), .b(\a[16] ), .c(\b[15] ), .out0(new_n181));
  nanp03aa1n02x5               g086(.a(new_n177), .b(new_n179), .c(new_n181), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[18] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[17] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\b[16] ), .clkout(new_n186));
  oaoi03aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  nona23aa1n02x4               g093(.a(new_n175), .b(new_n161), .c(new_n138), .d(new_n147), .out0(new_n189));
  aoi012aa1n02x5               g094(.a(new_n189), .b(new_n130), .c(new_n136), .o1(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(new_n175), .clkout(new_n191));
  aoai13aa1n02x5               g096(.a(new_n181), .b(new_n191), .c(new_n164), .d(new_n165), .o1(new_n192));
  xroi22aa1d04x5               g097(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n193));
  oai012aa1n02x5               g098(.a(new_n193), .b(new_n192), .c(new_n190), .o1(new_n194));
  oai022aa1n02x5               g099(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n195));
  oaib12aa1n02x5               g100(.a(new_n195), .b(new_n184), .c(\b[17] ), .out0(new_n196));
  norp02aa1n02x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n199), .clkout(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n194), .c(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n197), .clkout(new_n203));
  aoi012aa1n02x5               g108(.a(new_n199), .b(new_n194), .c(new_n196), .o1(new_n204));
  norp02aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  nano22aa1n02x4               g112(.a(new_n204), .b(new_n203), .c(new_n207), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n186), .b(new_n185), .o1(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .c(new_n209), .o1(new_n210));
  aoai13aa1n02x5               g115(.a(new_n200), .b(new_n210), .c(new_n182), .d(new_n193), .o1(new_n211));
  aoi012aa1n02x5               g116(.a(new_n207), .b(new_n211), .c(new_n203), .o1(new_n212));
  norp02aa1n02x5               g117(.a(new_n212), .b(new_n208), .o1(\s[20] ));
  nano23aa1n02x4               g118(.a(new_n197), .b(new_n205), .c(new_n206), .d(new_n198), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n193), .b(new_n214), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  oai012aa1n02x5               g121(.a(new_n216), .b(new_n192), .c(new_n190), .o1(new_n217));
  nona23aa1n02x4               g122(.a(new_n206), .b(new_n198), .c(new_n197), .d(new_n205), .out0(new_n218));
  aoi012aa1n02x5               g123(.a(new_n205), .b(new_n197), .c(new_n206), .o1(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n218), .c(new_n196), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n220), .clkout(new_n221));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  xnbna2aa1n03x5               g129(.a(new_n224), .b(new_n217), .c(new_n221), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n222), .clkout(new_n226));
  aobi12aa1n02x5               g131(.a(new_n224), .b(new_n217), .c(new_n221), .out0(new_n227));
  xnrc02aa1n02x5               g132(.a(\b[21] ), .b(\a[22] ), .out0(new_n228));
  nano22aa1n02x4               g133(.a(new_n227), .b(new_n226), .c(new_n228), .out0(new_n229));
  aoai13aa1n02x5               g134(.a(new_n224), .b(new_n220), .c(new_n182), .d(new_n216), .o1(new_n230));
  aoi012aa1n02x5               g135(.a(new_n228), .b(new_n230), .c(new_n226), .o1(new_n231));
  norp02aa1n02x5               g136(.a(new_n231), .b(new_n229), .o1(\s[22] ));
  nano22aa1n02x4               g137(.a(new_n228), .b(new_n226), .c(new_n223), .out0(new_n233));
  and003aa1n02x5               g138(.a(new_n193), .b(new_n233), .c(new_n214), .o(new_n234));
  oai012aa1n02x5               g139(.a(new_n234), .b(new_n192), .c(new_n190), .o1(new_n235));
  oao003aa1n02x5               g140(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .carry(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  aoi012aa1n02x5               g142(.a(new_n237), .b(new_n220), .c(new_n233), .o1(new_n238));
  xnrc02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .out0(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  xnbna2aa1n03x5               g145(.a(new_n240), .b(new_n235), .c(new_n238), .out0(\s[23] ));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  aoi012aa1n02x5               g148(.a(new_n239), .b(new_n235), .c(new_n238), .o1(new_n244));
  xnrc02aa1n02x5               g149(.a(\b[23] ), .b(\a[24] ), .out0(new_n245));
  nano22aa1n02x4               g150(.a(new_n244), .b(new_n243), .c(new_n245), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n238), .clkout(new_n247));
  aoai13aa1n02x5               g152(.a(new_n240), .b(new_n247), .c(new_n182), .d(new_n234), .o1(new_n248));
  aoi012aa1n02x5               g153(.a(new_n245), .b(new_n248), .c(new_n243), .o1(new_n249));
  norp02aa1n02x5               g154(.a(new_n249), .b(new_n246), .o1(\s[24] ));
  norp02aa1n02x5               g155(.a(new_n245), .b(new_n239), .o1(new_n251));
  nano22aa1n02x4               g156(.a(new_n215), .b(new_n233), .c(new_n251), .out0(new_n252));
  oai012aa1n02x5               g157(.a(new_n252), .b(new_n192), .c(new_n190), .o1(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n219), .clkout(new_n254));
  aoai13aa1n02x5               g159(.a(new_n233), .b(new_n254), .c(new_n214), .d(new_n210), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n251), .clkout(new_n256));
  oao003aa1n02x5               g161(.a(\a[24] ), .b(\b[23] ), .c(new_n243), .carry(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n256), .c(new_n255), .d(new_n236), .o1(new_n258));
  xnrc02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .out0(new_n259));
  aoib12aa1n02x5               g164(.a(new_n259), .b(new_n253), .c(new_n258), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n259), .clkout(new_n261));
  aoi112aa1n02x5               g166(.a(new_n261), .b(new_n258), .c(new_n182), .d(new_n252), .o1(new_n262));
  norp02aa1n02x5               g167(.a(new_n260), .b(new_n262), .o1(\s[25] ));
  norp02aa1n02x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[25] ), .b(\a[26] ), .out0(new_n266));
  nano22aa1n02x4               g171(.a(new_n260), .b(new_n265), .c(new_n266), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n261), .b(new_n258), .c(new_n182), .d(new_n252), .o1(new_n268));
  aoi012aa1n02x5               g173(.a(new_n266), .b(new_n268), .c(new_n265), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n269), .b(new_n267), .o1(\s[26] ));
  norp02aa1n02x5               g175(.a(new_n266), .b(new_n259), .o1(new_n271));
  nano32aa1n02x4               g176(.a(new_n215), .b(new_n271), .c(new_n233), .d(new_n251), .out0(new_n272));
  oai012aa1n02x5               g177(.a(new_n272), .b(new_n192), .c(new_n190), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n265), .carry(new_n274));
  aobi12aa1n02x5               g179(.a(new_n274), .b(new_n258), .c(new_n271), .out0(new_n275));
  xorc02aa1n02x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n273), .c(new_n275), .out0(\s[27] ));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n278), .clkout(new_n279));
  aobi12aa1n02x5               g184(.a(new_n276), .b(new_n273), .c(new_n275), .out0(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n280), .b(new_n279), .c(new_n281), .out0(new_n282));
  aoai13aa1n02x5               g187(.a(new_n251), .b(new_n237), .c(new_n220), .d(new_n233), .o1(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n271), .clkout(new_n284));
  aoai13aa1n02x5               g189(.a(new_n274), .b(new_n284), .c(new_n283), .d(new_n257), .o1(new_n285));
  aoai13aa1n02x5               g190(.a(new_n276), .b(new_n285), .c(new_n182), .d(new_n272), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n281), .b(new_n286), .c(new_n279), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n282), .o1(\s[28] ));
  norb02aa1n02x5               g193(.a(new_n276), .b(new_n281), .out0(new_n289));
  aobi12aa1n02x5               g194(.a(new_n289), .b(new_n273), .c(new_n275), .out0(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n290), .b(new_n291), .c(new_n292), .out0(new_n293));
  aoai13aa1n02x5               g198(.a(new_n289), .b(new_n285), .c(new_n182), .d(new_n272), .o1(new_n294));
  aoi012aa1n02x5               g199(.a(new_n292), .b(new_n294), .c(new_n291), .o1(new_n295));
  norp02aa1n02x5               g200(.a(new_n295), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g202(.a(new_n276), .b(new_n292), .c(new_n281), .out0(new_n298));
  aobi12aa1n02x5               g203(.a(new_n298), .b(new_n273), .c(new_n275), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[29] ), .b(\a[30] ), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n299), .b(new_n300), .c(new_n301), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n298), .b(new_n285), .c(new_n182), .d(new_n272), .o1(new_n303));
  aoi012aa1n02x5               g208(.a(new_n301), .b(new_n303), .c(new_n300), .o1(new_n304));
  norp02aa1n02x5               g209(.a(new_n304), .b(new_n302), .o1(\s[30] ));
  norb02aa1n02x5               g210(.a(new_n298), .b(new_n301), .out0(new_n306));
  aobi12aa1n02x5               g211(.a(new_n306), .b(new_n273), .c(new_n275), .out0(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n300), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n307), .b(new_n308), .c(new_n309), .out0(new_n310));
  aoai13aa1n02x5               g215(.a(new_n306), .b(new_n285), .c(new_n182), .d(new_n272), .o1(new_n311));
  aoi012aa1n02x5               g216(.a(new_n309), .b(new_n311), .c(new_n308), .o1(new_n312));
  norp02aa1n02x5               g217(.a(new_n312), .b(new_n310), .o1(\s[31] ));
  xnrb03aa1n02x5               g218(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaib12aa1n02x5               g222(.a(new_n134), .b(new_n105), .c(new_n117), .out0(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoib12aa1n02x5               g224(.a(new_n135), .b(new_n318), .c(new_n104), .out0(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g226(.a(\a[7] ), .b(\b[6] ), .c(new_n320), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g228(.a(new_n124), .b(new_n130), .c(new_n136), .out0(\s[9] ));
endmodule


