// Benchmark "adder" written by ABC on Thu Jul 11 11:43:52 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n320,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  oai012aa1n02x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
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
  norp02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanb02aa1n02x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  norb03aa1n02x5               g021(.a(new_n113), .b(new_n112), .c(new_n116), .out0(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[6] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[5] ), .clkout(new_n119));
  oaoi03aa1n02x5               g024(.a(new_n118), .b(new_n119), .c(new_n114), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n121), .b(new_n112), .c(new_n120), .o1(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n117), .c(new_n107), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .c(new_n123), .o1(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nano23aa1n02x4               g034(.a(new_n126), .b(new_n128), .c(new_n129), .d(new_n127), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n122), .c(new_n117), .d(new_n107), .o1(new_n131));
  aoi012aa1n02x5               g036(.a(new_n126), .b(new_n128), .c(new_n127), .o1(new_n132));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n131), .c(new_n132), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n133), .clkout(new_n137));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n123), .clkout(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n132), .clkout(new_n139));
  aoai13aa1n02x5               g044(.a(new_n135), .b(new_n139), .c(new_n138), .d(new_n130), .o1(new_n140));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n137), .out0(\s[12] ));
  nona23aa1n02x4               g049(.a(new_n142), .b(new_n134), .c(new_n133), .d(new_n141), .out0(new_n145));
  oaoi03aa1n02x5               g050(.a(\a[12] ), .b(\b[11] ), .c(new_n137), .o1(new_n146));
  oabi12aa1n02x5               g051(.a(new_n146), .b(new_n145), .c(new_n132), .out0(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  norb02aa1n02x5               g053(.a(new_n130), .b(new_n145), .out0(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n122), .c(new_n117), .d(new_n107), .o1(new_n150));
  norp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(new_n153));
  xobna2aa1n03x5               g058(.a(new_n153), .b(new_n150), .c(new_n148), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n151), .clkout(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n153), .c(new_n150), .d(new_n148), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nano23aa1n02x4               g064(.a(new_n151), .b(new_n158), .c(new_n159), .d(new_n152), .out0(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  oai012aa1n02x5               g066(.a(new_n159), .b(new_n158), .c(new_n151), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n161), .c(new_n150), .d(new_n148), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  and002aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o(new_n166));
  nona22aa1n02x4               g071(.a(new_n163), .b(new_n166), .c(new_n165), .out0(new_n167));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  oai112aa1n02x5               g075(.a(new_n167), .b(new_n170), .c(\b[14] ), .d(\a[15] ), .o1(new_n171));
  oaoi13aa1n02x5               g076(.a(new_n170), .b(new_n167), .c(\a[15] ), .d(\b[14] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[16] ));
  nano23aa1n02x4               g078(.a(new_n133), .b(new_n141), .c(new_n142), .d(new_n134), .out0(new_n174));
  nanp02aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nano23aa1n02x4               g080(.a(new_n165), .b(new_n168), .c(new_n169), .d(new_n175), .out0(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(new_n176), .clkout(new_n177));
  nano32aa1n02x4               g082(.a(new_n177), .b(new_n160), .c(new_n174), .d(new_n130), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n122), .c(new_n107), .d(new_n117), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n162), .clkout(new_n180));
  aoai13aa1n02x5               g085(.a(new_n176), .b(new_n180), .c(new_n147), .d(new_n160), .o1(new_n181));
  oai012aa1n02x5               g086(.a(new_n169), .b(new_n168), .c(new_n165), .o1(new_n182));
  nanp03aa1n02x5               g087(.a(new_n179), .b(new_n181), .c(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  norp02aa1n02x5               g089(.a(\b[16] ), .b(\a[17] ), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  oai012aa1n02x5               g091(.a(new_n186), .b(new_n183), .c(new_n185), .o1(new_n187));
  xnrb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  aoai13aa1n02x5               g093(.a(new_n160), .b(new_n146), .c(new_n174), .d(new_n139), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(new_n189), .b(new_n162), .o1(new_n190));
  aobi12aa1n02x5               g095(.a(new_n182), .b(new_n190), .c(new_n176), .out0(new_n191));
  norp02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  nano23aa1n02x4               g098(.a(new_n185), .b(new_n192), .c(new_n193), .d(new_n186), .out0(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(new_n194), .clkout(new_n195));
  aoi012aa1n02x5               g100(.a(new_n192), .b(new_n185), .c(new_n193), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n195), .c(new_n191), .d(new_n179), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  norp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  aoi112aa1n02x5               g110(.a(new_n200), .b(new_n205), .c(new_n197), .d(new_n202), .o1(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n200), .clkout(new_n207));
  160nm_ficinv00aa1n08x5       g112(.clk(new_n196), .clkout(new_n208));
  aoai13aa1n02x5               g113(.a(new_n202), .b(new_n208), .c(new_n183), .d(new_n194), .o1(new_n209));
  aobi12aa1n02x5               g114(.a(new_n205), .b(new_n209), .c(new_n207), .out0(new_n210));
  norp02aa1n02x5               g115(.a(new_n210), .b(new_n206), .o1(\s[20] ));
  nano23aa1n02x4               g116(.a(new_n200), .b(new_n203), .c(new_n204), .d(new_n201), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n212), .b(new_n194), .o1(new_n213));
  nona23aa1n02x4               g118(.a(new_n204), .b(new_n201), .c(new_n200), .d(new_n203), .out0(new_n214));
  oaoi03aa1n02x5               g119(.a(\a[20] ), .b(\b[19] ), .c(new_n207), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  oai012aa1n02x5               g121(.a(new_n216), .b(new_n214), .c(new_n196), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n213), .c(new_n191), .d(new_n179), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoi112aa1n02x5               g128(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n221), .clkout(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n213), .clkout(new_n226));
  aoai13aa1n02x5               g131(.a(new_n222), .b(new_n217), .c(new_n183), .d(new_n226), .o1(new_n227));
  aobi12aa1n02x5               g132(.a(new_n223), .b(new_n227), .c(new_n225), .out0(new_n228));
  norp02aa1n02x5               g133(.a(new_n228), .b(new_n224), .o1(\s[22] ));
  nano22aa1n02x4               g134(.a(new_n213), .b(new_n222), .c(new_n223), .out0(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n230), .clkout(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(\a[21] ), .clkout(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(\a[22] ), .clkout(new_n233));
  xroi22aa1d04x5               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  oaoi03aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n217), .c(new_n234), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n231), .c(new_n191), .d(new_n179), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoi112aa1n02x5               g146(.a(new_n239), .b(new_n241), .c(new_n237), .d(new_n240), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n239), .clkout(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n236), .clkout(new_n244));
  aoai13aa1n02x5               g149(.a(new_n240), .b(new_n244), .c(new_n183), .d(new_n230), .o1(new_n245));
  aobi12aa1n02x5               g150(.a(new_n241), .b(new_n245), .c(new_n243), .out0(new_n246));
  norp02aa1n02x5               g151(.a(new_n246), .b(new_n242), .o1(\s[24] ));
  and002aa1n02x5               g152(.a(new_n241), .b(new_n240), .o(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  nano32aa1n02x4               g154(.a(new_n249), .b(new_n234), .c(new_n212), .d(new_n194), .out0(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n250), .clkout(new_n251));
  aoai13aa1n02x5               g156(.a(new_n234), .b(new_n215), .c(new_n212), .d(new_n208), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n235), .clkout(new_n253));
  nanp02aa1n02x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  oai022aa1n02x5               g159(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n255), .b(new_n254), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n249), .c(new_n252), .d(new_n253), .o1(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n257), .clkout(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n251), .c(new_n191), .d(new_n179), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xorc02aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  aoi112aa1n02x5               g168(.a(new_n261), .b(new_n263), .c(new_n259), .d(new_n262), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n261), .clkout(new_n265));
  aoai13aa1n02x5               g170(.a(new_n262), .b(new_n257), .c(new_n183), .d(new_n250), .o1(new_n266));
  aobi12aa1n02x5               g171(.a(new_n263), .b(new_n266), .c(new_n265), .out0(new_n267));
  norp02aa1n02x5               g172(.a(new_n267), .b(new_n264), .o1(\s[26] ));
  aoai13aa1n02x5               g173(.a(new_n182), .b(new_n177), .c(new_n189), .d(new_n162), .o1(new_n269));
  and002aa1n02x5               g174(.a(new_n263), .b(new_n262), .o(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  nano23aa1n02x4               g176(.a(new_n213), .b(new_n271), .c(new_n248), .d(new_n234), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n269), .c(new_n138), .d(new_n178), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n265), .carry(new_n274));
  aobi12aa1n02x5               g179(.a(new_n274), .b(new_n257), .c(new_n270), .out0(new_n275));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  norb02aa1n02x5               g182(.a(new_n277), .b(new_n276), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n273), .c(new_n275), .out0(\s[27] ));
  xorc02aa1n02x5               g184(.a(\a[28] ), .b(\b[27] ), .out0(new_n280));
  aoai13aa1n02x5               g185(.a(new_n248), .b(new_n235), .c(new_n217), .d(new_n234), .o1(new_n281));
  aoai13aa1n02x5               g186(.a(new_n274), .b(new_n271), .c(new_n281), .d(new_n256), .o1(new_n282));
  aoi112aa1n02x5               g187(.a(new_n282), .b(new_n276), .c(new_n183), .d(new_n272), .o1(new_n283));
  nano22aa1n02x4               g188(.a(new_n283), .b(new_n277), .c(new_n280), .out0(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n276), .clkout(new_n285));
  nanp03aa1n02x5               g190(.a(new_n273), .b(new_n275), .c(new_n285), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n280), .b(new_n286), .c(new_n277), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n284), .o1(\s[28] ));
  and002aa1n02x5               g193(.a(new_n280), .b(new_n278), .o(new_n289));
  aoai13aa1n02x5               g194(.a(new_n289), .b(new_n282), .c(new_n183), .d(new_n272), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n285), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  aobi12aa1n02x5               g198(.a(new_n289), .b(new_n273), .c(new_n275), .out0(new_n294));
  nano22aa1n02x4               g199(.a(new_n294), .b(new_n291), .c(new_n292), .out0(new_n295));
  norp02aa1n02x5               g200(.a(new_n293), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g202(.a(new_n292), .b(new_n280), .c(new_n278), .out0(new_n298));
  aoai13aa1n02x5               g203(.a(new_n298), .b(new_n282), .c(new_n183), .d(new_n272), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[29] ), .b(\a[30] ), .out0(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(new_n299), .c(new_n300), .o1(new_n302));
  aobi12aa1n02x5               g207(.a(new_n298), .b(new_n273), .c(new_n275), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n303), .b(new_n300), .c(new_n301), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n302), .b(new_n304), .o1(\s[30] ));
  nano23aa1n02x4               g210(.a(new_n301), .b(new_n292), .c(new_n280), .d(new_n278), .out0(new_n306));
  aobi12aa1n02x5               g211(.a(new_n306), .b(new_n273), .c(new_n275), .out0(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n300), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n307), .b(new_n308), .c(new_n309), .out0(new_n310));
  aoai13aa1n02x5               g215(.a(new_n306), .b(new_n282), .c(new_n183), .d(new_n272), .o1(new_n311));
  aoi012aa1n02x5               g216(.a(new_n309), .b(new_n311), .c(new_n308), .o1(new_n312));
  norp02aa1n02x5               g217(.a(new_n312), .b(new_n310), .o1(\s[31] ));
  xnrb03aa1n02x5               g218(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g222(.a(new_n115), .b(new_n107), .c(new_n114), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(new_n118), .out0(\s[6] ));
  oaoi03aa1n02x5               g224(.a(\a[6] ), .b(\b[5] ), .c(new_n318), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n110), .b(new_n320), .c(new_n111), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g228(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


