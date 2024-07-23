// Benchmark "adder" written by ABC on Thu Jul 11 11:13:27 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n308, new_n310, new_n311;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[9] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[8] ), .clkout(new_n99));
  nanp02aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  norb02aa1n02x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  160nm_ficinv00aa1n08x5       g009(.clk(new_n104), .clkout(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(new_n107), .b(new_n106), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(new_n108), .b(new_n105), .o1(new_n109));
  oab012aa1n02x4               g014(.a(new_n101), .b(\a[4] ), .c(\b[3] ), .out0(new_n110));
  160nm_ficinv00aa1n08x5       g015(.clk(new_n110), .clkout(new_n111));
  aoai13aa1n02x5               g016(.a(new_n100), .b(new_n111), .c(new_n109), .d(new_n103), .o1(new_n112));
  xorc02aa1n02x5               g017(.a(\a[8] ), .b(\b[7] ), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[7] ), .b(\b[6] ), .out0(new_n114));
  nanp02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nano23aa1n02x4               g023(.a(new_n117), .b(new_n116), .c(new_n118), .d(new_n115), .out0(new_n119));
  nanp03aa1n02x5               g024(.a(new_n119), .b(new_n113), .c(new_n114), .o1(new_n120));
  aoi012aa1n02x5               g025(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(new_n121), .clkout(new_n122));
  orn002aa1n02x5               g027(.a(\a[7] ), .b(\b[6] ), .o(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  aoi013aa1n02x4               g029(.a(new_n124), .b(new_n122), .c(new_n114), .d(new_n113), .o1(new_n125));
  oai012aa1n02x5               g030(.a(new_n125), .b(new_n112), .c(new_n120), .o1(new_n126));
  oaoi03aa1n02x5               g031(.a(new_n98), .b(new_n99), .c(new_n126), .o1(new_n127));
  xnrc02aa1n02x5               g032(.a(new_n127), .b(new_n97), .out0(\s[10] ));
  oaoi03aa1n02x5               g033(.a(\a[10] ), .b(\b[9] ), .c(new_n127), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  norp02aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n131), .c(new_n129), .d(new_n133), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n129), .b(new_n133), .o1(new_n138));
  nona22aa1n02x4               g043(.a(new_n138), .b(new_n136), .c(new_n131), .out0(new_n139));
  nanp02aa1n02x5               g044(.a(new_n139), .b(new_n137), .o1(\s[12] ));
  nano23aa1n02x4               g045(.a(new_n131), .b(new_n134), .c(new_n135), .d(new_n132), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n99), .b(new_n98), .o1(new_n142));
  oaoi03aa1n02x5               g047(.a(\a[10] ), .b(\b[9] ), .c(new_n142), .o1(new_n143));
  160nm_fiao0012aa1n02p5x5     g048(.a(new_n134), .b(new_n131), .c(new_n135), .o(new_n144));
  aoi012aa1n02x5               g049(.a(new_n144), .b(new_n141), .c(new_n143), .o1(new_n145));
  aoai13aa1n02x5               g050(.a(new_n103), .b(new_n104), .c(new_n107), .d(new_n106), .o1(new_n146));
  aoi022aa1n02x5               g051(.a(new_n146), .b(new_n110), .c(\b[3] ), .d(\a[4] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n114), .b(new_n113), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n119), .b(new_n148), .out0(new_n149));
  oabi12aa1n02x5               g054(.a(new_n124), .b(new_n148), .c(new_n121), .out0(new_n150));
  xorc02aa1n02x5               g055(.a(\a[9] ), .b(\b[8] ), .out0(new_n151));
  nanp03aa1n02x5               g056(.a(new_n141), .b(new_n97), .c(new_n151), .o1(new_n152));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n152), .clkout(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n150), .c(new_n147), .d(new_n149), .o1(new_n154));
  xnrc02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n154), .c(new_n145), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g062(.clk(\a[14] ), .clkout(new_n158));
  nanp02aa1n02x5               g063(.a(new_n154), .b(new_n145), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n156), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(new_n158), .out0(\s[14] ));
  xnrc02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .out0(new_n163));
  norp02aa1n02x5               g068(.a(new_n163), .b(new_n155), .o1(new_n164));
  160nm_ficinv00aa1n08x5       g069(.clk(new_n164), .clkout(new_n165));
  160nm_ficinv00aa1n08x5       g070(.clk(\b[13] ), .clkout(new_n166));
  oaoi03aa1n02x5               g071(.a(new_n158), .b(new_n166), .c(new_n160), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n165), .c(new_n154), .d(new_n145), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  xnrc02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .out0(new_n171));
  160nm_ficinv00aa1n08x5       g076(.clk(new_n171), .clkout(new_n172));
  xnrc02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n170), .c(new_n168), .d(new_n172), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(new_n168), .b(new_n172), .o1(new_n175));
  nona22aa1n02x4               g080(.a(new_n175), .b(new_n173), .c(new_n170), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n174), .o1(\s[16] ));
  norp02aa1n02x5               g082(.a(new_n173), .b(new_n171), .o1(new_n178));
  nano22aa1n02x4               g083(.a(new_n152), .b(new_n164), .c(new_n178), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n150), .c(new_n147), .d(new_n149), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n178), .b(new_n164), .o1(new_n181));
  orn002aa1n02x5               g086(.a(\a[15] ), .b(\b[14] ), .o(new_n182));
  oao003aa1n02x5               g087(.a(\a[16] ), .b(\b[15] ), .c(new_n182), .carry(new_n183));
  oai013aa1n02x4               g088(.a(new_n183), .b(new_n167), .c(new_n171), .d(new_n173), .o1(new_n184));
  oab012aa1n02x4               g089(.a(new_n184), .b(new_n181), .c(new_n145), .out0(new_n185));
  xorc02aa1n02x5               g090(.a(\a[17] ), .b(\b[16] ), .out0(new_n186));
  xnbna2aa1n03x5               g091(.a(new_n186), .b(new_n180), .c(new_n185), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[18] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\a[17] ), .clkout(new_n189));
  aobi12aa1n02x5               g094(.a(new_n186), .b(new_n180), .c(new_n185), .out0(new_n190));
  aoib12aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(\b[16] ), .out0(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  norp02aa1n02x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n167), .clkout(new_n194));
  aobi12aa1n02x5               g099(.a(new_n183), .b(new_n178), .c(new_n194), .out0(new_n195));
  oai012aa1n02x5               g100(.a(new_n195), .b(new_n181), .c(new_n145), .o1(new_n196));
  xroi22aa1d04x5               g101(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n196), .c(new_n126), .d(new_n179), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n199));
  nona22aa1n02x4               g104(.a(new_n198), .b(new_n199), .c(new_n193), .out0(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  norp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n203), .c(new_n200), .d(new_n205), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(new_n200), .b(new_n205), .o1(new_n211));
  nona22aa1n02x4               g116(.a(new_n211), .b(new_n209), .c(new_n203), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n212), .b(new_n210), .o1(\s[20] ));
  oai112aa1n02x5               g118(.a(new_n205), .b(new_n208), .c(new_n199), .d(new_n193), .o1(new_n214));
  aoi012aa1n02x5               g119(.a(new_n206), .b(new_n203), .c(new_n207), .o1(new_n215));
  and002aa1n02x5               g120(.a(new_n214), .b(new_n215), .o(new_n216));
  nano23aa1n02x4               g121(.a(new_n203), .b(new_n206), .c(new_n207), .d(new_n204), .out0(new_n217));
  nanp02aa1n02x5               g122(.a(new_n197), .b(new_n217), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n180), .d(new_n185), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xnrc02aa1n02x5               g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(new_n219), .b(new_n222), .o1(new_n225));
  nona22aa1n02x4               g130(.a(new_n225), .b(new_n223), .c(new_n221), .out0(new_n226));
  nanp02aa1n02x5               g131(.a(new_n226), .b(new_n224), .o1(\s[22] ));
  nanb02aa1n02x5               g132(.a(new_n223), .b(new_n222), .out0(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(\a[22] ), .clkout(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\b[21] ), .clkout(new_n230));
  oaoi03aa1n02x5               g135(.a(new_n229), .b(new_n230), .c(new_n221), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n228), .c(new_n214), .d(new_n215), .o1(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  norb02aa1n02x5               g138(.a(new_n222), .b(new_n223), .out0(new_n234));
  nanp03aa1n02x5               g139(.a(new_n234), .b(new_n197), .c(new_n217), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n180), .d(new_n185), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  xorc02aa1n02x5               g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  xnrc02aa1n02x5               g144(.a(\b[23] ), .b(\a[24] ), .out0(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n238), .c(new_n236), .d(new_n239), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(new_n236), .b(new_n239), .o1(new_n242));
  nona22aa1n02x4               g147(.a(new_n242), .b(new_n240), .c(new_n238), .out0(new_n243));
  nanp02aa1n02x5               g148(.a(new_n243), .b(new_n241), .o1(\s[24] ));
  norb02aa1n02x5               g149(.a(new_n239), .b(new_n240), .out0(new_n245));
  nanb03aa1n02x5               g150(.a(new_n218), .b(new_n245), .c(new_n234), .out0(new_n246));
  orn002aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .o(new_n247));
  oao003aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .c(new_n247), .carry(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  aoi012aa1n02x5               g154(.a(new_n249), .b(new_n232), .c(new_n245), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n246), .c(new_n180), .d(new_n185), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xorc02aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n255), .clkout(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n251), .b(new_n254), .o1(new_n258));
  nona22aa1n02x4               g163(.a(new_n258), .b(new_n256), .c(new_n253), .out0(new_n259));
  nanp02aa1n02x5               g164(.a(new_n259), .b(new_n257), .o1(\s[26] ));
  and002aa1n02x5               g165(.a(new_n255), .b(new_n254), .o(new_n261));
  nano22aa1n02x4               g166(.a(new_n235), .b(new_n245), .c(new_n261), .out0(new_n262));
  aoai13aa1n02x5               g167(.a(new_n262), .b(new_n196), .c(new_n126), .d(new_n179), .o1(new_n263));
  aoai13aa1n02x5               g168(.a(new_n261), .b(new_n249), .c(new_n232), .d(new_n245), .o1(new_n264));
  aoi112aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n265));
  oab012aa1n02x4               g170(.a(new_n265), .b(\a[26] ), .c(\b[25] ), .out0(new_n266));
  nanp03aa1n02x5               g171(.a(new_n263), .b(new_n264), .c(new_n266), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[27] ), .b(\a[28] ), .out0(new_n271));
  aoai13aa1n02x5               g176(.a(new_n271), .b(new_n269), .c(new_n267), .d(new_n270), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n262), .clkout(new_n273));
  aoi012aa1n02x5               g178(.a(new_n273), .b(new_n180), .c(new_n185), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(new_n264), .b(new_n266), .o1(new_n275));
  oai012aa1n02x5               g180(.a(new_n270), .b(new_n275), .c(new_n274), .o1(new_n276));
  nona22aa1n02x4               g181(.a(new_n276), .b(new_n271), .c(new_n269), .out0(new_n277));
  nanp02aa1n02x5               g182(.a(new_n272), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g183(.a(new_n270), .b(new_n271), .out0(new_n279));
  oai012aa1n02x5               g184(.a(new_n279), .b(new_n275), .c(new_n274), .o1(new_n280));
  aob012aa1n02x5               g185(.a(new_n269), .b(\b[27] ), .c(\a[28] ), .out0(new_n281));
  oa0012aa1n02x5               g186(.a(new_n281), .b(\b[27] ), .c(\a[28] ), .o(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n282), .clkout(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  nona22aa1n02x4               g189(.a(new_n280), .b(new_n283), .c(new_n284), .out0(new_n285));
  aoai13aa1n02x5               g190(.a(new_n284), .b(new_n283), .c(new_n267), .d(new_n279), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(new_n286), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g193(.a(new_n270), .b(new_n284), .c(new_n271), .out0(new_n289));
  oaoi03aa1n02x5               g194(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .o1(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n290), .c(new_n267), .d(new_n289), .o1(new_n292));
  oai012aa1n02x5               g197(.a(new_n289), .b(new_n275), .c(new_n274), .o1(new_n293));
  nona22aa1n02x4               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  nanp02aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g200(.a(new_n289), .b(new_n291), .out0(new_n296));
  oai012aa1n02x5               g201(.a(new_n296), .b(new_n275), .c(new_n274), .o1(new_n297));
  nanb02aa1n02x5               g202(.a(new_n291), .b(new_n290), .out0(new_n298));
  oai012aa1n02x5               g203(.a(new_n298), .b(\b[29] ), .c(\a[30] ), .o1(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  nona22aa1n02x4               g205(.a(new_n297), .b(new_n299), .c(new_n300), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n300), .b(new_n299), .c(new_n267), .d(new_n296), .o1(new_n302));
  nanp02aa1n02x5               g207(.a(new_n302), .b(new_n301), .o1(\s[31] ));
  xnbna2aa1n03x5               g208(.a(new_n103), .b(new_n108), .c(new_n105), .out0(\s[3] ));
  aoi012aa1n02x5               g209(.a(new_n101), .b(new_n109), .c(new_n103), .o1(new_n305));
  xnrb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnrb03aa1n02x5               g211(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g212(.a(\a[5] ), .b(\b[4] ), .c(new_n112), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g214(.a(new_n114), .b(new_n122), .c(new_n147), .d(new_n119), .o1(new_n310));
  aoi112aa1n02x5               g215(.a(new_n114), .b(new_n122), .c(new_n147), .d(new_n119), .o1(new_n311));
  norb02aa1n02x5               g216(.a(new_n310), .b(new_n311), .out0(\s[7] ));
  xnbna2aa1n03x5               g217(.a(new_n113), .b(new_n310), .c(new_n123), .out0(\s[8] ));
  xorb03aa1n02x5               g218(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


