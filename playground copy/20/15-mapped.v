// Benchmark "adder" written by ABC on Thu Jul 11 12:21:45 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n321, new_n323, new_n324, new_n326;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  aoi112aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n119));
  oab012aa1n02x4               g024(.a(new_n119), .b(\a[6] ), .c(\b[5] ), .out0(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\a[7] ), .clkout(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\b[6] ), .clkout(new_n122));
  aoai13aa1n02x5               g027(.a(new_n111), .b(new_n110), .c(new_n121), .d(new_n122), .o1(new_n123));
  oai012aa1n02x5               g028(.a(new_n123), .b(new_n114), .c(new_n120), .o1(new_n124));
  160nm_ficinv00aa1n08x5       g029(.clk(new_n124), .clkout(new_n125));
  xnrc02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n98), .b(new_n126), .c(new_n118), .d(new_n125), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xnrc02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .out0(new_n129));
  160nm_ficinv00aa1n08x5       g034(.clk(new_n129), .clkout(new_n130));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  oaoi03aa1n02x5               g038(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n133), .b(new_n134), .c(new_n127), .d(new_n130), .o1(new_n135));
  aoi112aa1n02x5               g040(.a(new_n133), .b(new_n134), .c(new_n127), .d(new_n130), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n131), .clkout(new_n138));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n135), .c(new_n138), .out0(\s[12] ));
  nona23aa1n02x4               g047(.a(new_n140), .b(new_n132), .c(new_n131), .d(new_n139), .out0(new_n143));
  norp03aa1n02x5               g048(.a(new_n143), .b(new_n126), .c(new_n129), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n146));
  oab012aa1n02x4               g051(.a(new_n146), .b(\a[10] ), .c(\b[9] ), .out0(new_n147));
  oaoi03aa1n02x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n138), .o1(new_n148));
  oabi12aa1n02x5               g053(.a(new_n148), .b(new_n143), .c(new_n147), .out0(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  norp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(new_n153));
  xobna2aa1n03x5               g058(.a(new_n153), .b(new_n145), .c(new_n150), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n151), .clkout(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n153), .c(new_n145), .d(new_n150), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nano23aa1n02x4               g064(.a(new_n151), .b(new_n158), .c(new_n159), .d(new_n152), .out0(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  oai012aa1n02x5               g066(.a(new_n159), .b(new_n158), .c(new_n151), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n161), .c(new_n145), .d(new_n150), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  aoi012aa1n02x5               g071(.a(new_n165), .b(new_n163), .c(new_n166), .o1(new_n167));
  xnrb03aa1n02x5               g072(.a(new_n167), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  nano23aa1n02x4               g073(.a(new_n131), .b(new_n139), .c(new_n140), .d(new_n132), .out0(new_n169));
  nona22aa1n02x4               g074(.a(new_n169), .b(new_n126), .c(new_n129), .out0(new_n170));
  norp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nano23aa1n02x4               g077(.a(new_n165), .b(new_n171), .c(new_n172), .d(new_n166), .out0(new_n173));
  nano22aa1n02x4               g078(.a(new_n170), .b(new_n160), .c(new_n173), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(new_n162), .clkout(new_n176));
  aoai13aa1n02x5               g081(.a(new_n173), .b(new_n176), .c(new_n149), .d(new_n160), .o1(new_n177));
  oai012aa1n02x5               g082(.a(new_n172), .b(new_n171), .c(new_n165), .o1(new_n178));
  nanp03aa1n02x5               g083(.a(new_n175), .b(new_n177), .c(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nanp03aa1n02x5               g085(.a(new_n144), .b(new_n160), .c(new_n173), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n181), .b(new_n118), .c(new_n125), .o1(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n173), .clkout(new_n183));
  aoai13aa1n02x5               g088(.a(new_n160), .b(new_n148), .c(new_n169), .d(new_n134), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n178), .b(new_n183), .c(new_n184), .d(new_n162), .o1(new_n185));
  norp02aa1n02x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  oaoi13aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n185), .d(new_n182), .o1(new_n188));
  xnrb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  norp02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nano23aa1n02x4               g096(.a(new_n186), .b(new_n190), .c(new_n191), .d(new_n187), .out0(new_n192));
  oai012aa1n02x5               g097(.a(new_n192), .b(new_n185), .c(new_n182), .o1(new_n193));
  aoi012aa1n02x5               g098(.a(new_n190), .b(new_n186), .c(new_n191), .o1(new_n194));
  norp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n193), .c(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n195), .clkout(new_n200));
  aobi12aa1n02x5               g105(.a(new_n197), .b(new_n193), .c(new_n194), .out0(new_n201));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n204), .clkout(new_n205));
  nano22aa1n02x4               g110(.a(new_n201), .b(new_n200), .c(new_n205), .out0(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n194), .clkout(new_n207));
  aoai13aa1n02x5               g112(.a(new_n197), .b(new_n207), .c(new_n179), .d(new_n192), .o1(new_n208));
  aoi012aa1n02x5               g113(.a(new_n205), .b(new_n208), .c(new_n200), .o1(new_n209));
  norp02aa1n02x5               g114(.a(new_n209), .b(new_n206), .o1(\s[20] ));
  nanp02aa1n02x5               g115(.a(new_n184), .b(new_n162), .o1(new_n211));
  aobi12aa1n02x5               g116(.a(new_n178), .b(new_n211), .c(new_n173), .out0(new_n212));
  nano23aa1n02x4               g117(.a(new_n195), .b(new_n202), .c(new_n203), .d(new_n196), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n213), .b(new_n192), .o1(new_n214));
  nona23aa1n02x4               g119(.a(new_n203), .b(new_n196), .c(new_n195), .d(new_n202), .out0(new_n215));
  oaoi03aa1n02x5               g120(.a(\a[20] ), .b(\b[19] ), .c(new_n200), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  oai012aa1n02x5               g122(.a(new_n217), .b(new_n215), .c(new_n194), .o1(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n214), .c(new_n212), .d(new_n175), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[21] ), .b(\b[20] ), .out0(new_n223));
  xorc02aa1n02x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n222), .b(new_n224), .c(new_n220), .d(new_n223), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n222), .clkout(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n214), .clkout(new_n227));
  aoai13aa1n02x5               g132(.a(new_n223), .b(new_n218), .c(new_n179), .d(new_n227), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n224), .clkout(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n228), .c(new_n226), .o1(new_n230));
  norp02aa1n02x5               g135(.a(new_n230), .b(new_n225), .o1(\s[22] ));
  nano22aa1n02x4               g136(.a(new_n214), .b(new_n223), .c(new_n224), .out0(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\a[21] ), .clkout(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(\a[22] ), .clkout(new_n235));
  xroi22aa1d04x5               g140(.a(new_n234), .b(\b[20] ), .c(new_n235), .d(\b[21] ), .out0(new_n236));
  oaoi03aa1n02x5               g141(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .o1(new_n237));
  aoi012aa1n02x5               g142(.a(new_n237), .b(new_n218), .c(new_n236), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n233), .c(new_n212), .d(new_n175), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  xorc02aa1n02x5               g146(.a(\a[23] ), .b(\b[22] ), .out0(new_n242));
  xorc02aa1n02x5               g147(.a(\a[24] ), .b(\b[23] ), .out0(new_n243));
  aoi112aa1n02x5               g148(.a(new_n241), .b(new_n243), .c(new_n239), .d(new_n242), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n241), .clkout(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n238), .clkout(new_n246));
  aoai13aa1n02x5               g151(.a(new_n242), .b(new_n246), .c(new_n179), .d(new_n232), .o1(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n243), .clkout(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n247), .c(new_n245), .o1(new_n249));
  norp02aa1n02x5               g154(.a(new_n249), .b(new_n244), .o1(\s[24] ));
  and002aa1n02x5               g155(.a(new_n243), .b(new_n242), .o(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n251), .clkout(new_n252));
  nano32aa1n02x4               g157(.a(new_n252), .b(new_n236), .c(new_n213), .d(new_n192), .out0(new_n253));
  oai012aa1n02x5               g158(.a(new_n253), .b(new_n185), .c(new_n182), .o1(new_n254));
  aoai13aa1n02x5               g159(.a(new_n236), .b(new_n216), .c(new_n213), .d(new_n207), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n237), .clkout(new_n256));
  nanp02aa1n02x5               g161(.a(\b[23] ), .b(\a[24] ), .o1(new_n257));
  oai022aa1n02x5               g162(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n258), .b(new_n257), .o1(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n252), .c(new_n255), .d(new_n256), .o1(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n260), .clkout(new_n261));
  xorc02aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n254), .c(new_n261), .out0(\s[25] ));
  norp02aa1n02x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  aobi12aa1n02x5               g170(.a(new_n262), .b(new_n254), .c(new_n261), .out0(new_n266));
  xorc02aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .out0(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n267), .clkout(new_n268));
  nano22aa1n02x4               g173(.a(new_n266), .b(new_n265), .c(new_n268), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n262), .b(new_n260), .c(new_n179), .d(new_n253), .o1(new_n270));
  aoi012aa1n02x5               g175(.a(new_n268), .b(new_n270), .c(new_n265), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n269), .o1(\s[26] ));
  and002aa1n02x5               g177(.a(new_n267), .b(new_n262), .o(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  nano23aa1n02x4               g179(.a(new_n214), .b(new_n274), .c(new_n251), .d(new_n236), .out0(new_n275));
  oai012aa1n02x5               g180(.a(new_n275), .b(new_n185), .c(new_n182), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n265), .carry(new_n277));
  aobi12aa1n02x5               g182(.a(new_n277), .b(new_n260), .c(new_n273), .out0(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n278), .c(new_n276), .out0(\s[27] ));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n281), .clkout(new_n282));
  aobi12aa1n02x5               g187(.a(new_n279), .b(new_n278), .c(new_n276), .out0(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n283), .b(new_n282), .c(new_n284), .out0(new_n285));
  aoai13aa1n02x5               g190(.a(new_n251), .b(new_n237), .c(new_n218), .d(new_n236), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n277), .b(new_n274), .c(new_n286), .d(new_n259), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n279), .b(new_n287), .c(new_n179), .d(new_n275), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n284), .b(new_n288), .c(new_n282), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n289), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g195(.a(new_n279), .b(new_n284), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n287), .c(new_n179), .d(new_n275), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n02x5               g200(.a(new_n291), .b(new_n278), .c(new_n276), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n295), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g204(.a(new_n279), .b(new_n294), .c(new_n284), .out0(new_n300));
  aoai13aa1n02x5               g205(.a(new_n300), .b(new_n287), .c(new_n179), .d(new_n275), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  aoi012aa1n02x5               g208(.a(new_n303), .b(new_n301), .c(new_n302), .o1(new_n304));
  aobi12aa1n02x5               g209(.a(new_n300), .b(new_n278), .c(new_n276), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n305), .b(new_n302), .c(new_n303), .out0(new_n306));
  norp02aa1n02x5               g211(.a(new_n304), .b(new_n306), .o1(\s[30] ));
  norb02aa1n02x5               g212(.a(new_n300), .b(new_n303), .out0(new_n308));
  aobi12aa1n02x5               g213(.a(new_n308), .b(new_n278), .c(new_n276), .out0(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  nano22aa1n02x4               g216(.a(new_n309), .b(new_n310), .c(new_n311), .out0(new_n312));
  aoai13aa1n02x5               g217(.a(new_n308), .b(new_n287), .c(new_n179), .d(new_n275), .o1(new_n313));
  aoi012aa1n02x5               g218(.a(new_n311), .b(new_n313), .c(new_n310), .o1(new_n314));
  norp02aa1n02x5               g219(.a(new_n314), .b(new_n312), .o1(\s[31] ));
  xnrb03aa1n02x5               g220(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g224(.a(new_n116), .b(new_n109), .out0(new_n320));
  oai012aa1n02x5               g225(.a(new_n320), .b(\b[4] ), .c(\a[5] ), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  orn002aa1n02x5               g227(.a(\a[5] ), .b(\b[4] ), .o(new_n323));
  aoai13aa1n02x5               g228(.a(new_n120), .b(new_n115), .c(new_n320), .d(new_n323), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g230(.a(new_n121), .b(new_n122), .c(new_n324), .o1(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g232(.a(new_n126), .b(new_n118), .c(new_n125), .out0(\s[9] ));
endmodule


