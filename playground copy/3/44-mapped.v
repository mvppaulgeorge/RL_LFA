// Benchmark "adder" written by ABC on Wed Jul 10 16:57:04 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n304, new_n307, new_n308, new_n310, new_n311, new_n313;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  and002aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(new_n100), .clkout(new_n101));
  and002aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\a[3] ), .clkout(new_n103));
  160nm_ficinv00aa1n08x5       g008(.clk(\b[2] ), .clkout(new_n104));
  nanp02aa1n02x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  aoi012aa1n02x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  160nm_ficinv00aa1n08x5       g016(.clk(\a[4] ), .clkout(new_n112));
  aboi22aa1n03x5               g017(.a(\b[3] ), .b(new_n112), .c(new_n103), .d(new_n104), .out0(new_n113));
  oaoi13aa1n02x5               g018(.a(new_n102), .b(new_n113), .c(new_n111), .d(new_n107), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nona23aa1n02x4               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  xorc02aa1n02x5               g024(.a(\a[6] ), .b(\b[5] ), .out0(new_n120));
  xorc02aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .out0(new_n121));
  nano22aa1n02x4               g026(.a(new_n119), .b(new_n120), .c(new_n121), .out0(new_n122));
  nanp02aa1n02x5               g027(.a(new_n122), .b(new_n114), .o1(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(\b[5] ), .clkout(new_n124));
  oai022aa1n02x5               g029(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n125));
  oaib12aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(\a[6] ), .out0(new_n126));
  160nm_ficinv00aa1n08x5       g031(.clk(new_n117), .clkout(new_n127));
  oaoi03aa1n02x5               g032(.a(\a[8] ), .b(\b[7] ), .c(new_n127), .o1(new_n128));
  oabi12aa1n02x5               g033(.a(new_n128), .b(new_n119), .c(new_n126), .out0(new_n129));
  160nm_ficinv00aa1n08x5       g034(.clk(new_n129), .clkout(new_n130));
  xorc02aa1n02x5               g035(.a(\a[9] ), .b(\b[8] ), .out0(new_n131));
  nanp03aa1n02x5               g036(.a(new_n123), .b(new_n130), .c(new_n131), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n99), .b(new_n132), .c(new_n101), .out0(\s[10] ));
  norp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n134), .clkout(new_n135));
  nanp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n98), .b(new_n97), .c(new_n132), .d(new_n101), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n135), .c(new_n136), .out0(\s[11] ));
  nanb02aa1n02x5               g043(.a(new_n134), .b(new_n136), .out0(new_n139));
  xnrc02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .out0(new_n140));
  oai112aa1n02x5               g045(.a(new_n135), .b(new_n140), .c(new_n137), .d(new_n139), .o1(new_n141));
  oaoi13aa1n02x5               g046(.a(new_n140), .b(new_n135), .c(new_n137), .d(new_n139), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(\s[12] ));
  xnrc02aa1n02x5               g048(.a(\b[12] ), .b(\a[13] ), .out0(new_n144));
  xorc02aa1n02x5               g049(.a(\a[12] ), .b(\b[11] ), .out0(new_n145));
  nano22aa1n02x4               g050(.a(new_n134), .b(new_n98), .c(new_n136), .out0(new_n146));
  oai022aa1n02x5               g051(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n147));
  nona23aa1n02x4               g052(.a(new_n146), .b(new_n145), .c(new_n147), .d(new_n100), .out0(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n129), .c(new_n122), .d(new_n114), .o1(new_n150));
  oao003aa1n02x5               g055(.a(\a[12] ), .b(\b[11] ), .c(new_n135), .carry(new_n151));
  160nm_ficinv00aa1n08x5       g056(.clk(new_n151), .clkout(new_n152));
  aoi013aa1n02x4               g057(.a(new_n152), .b(new_n146), .c(new_n145), .d(new_n147), .o1(new_n153));
  xobna2aa1n03x5               g058(.a(new_n144), .b(new_n150), .c(new_n153), .out0(\s[13] ));
  orn002aa1n02x5               g059(.a(\a[13] ), .b(\b[12] ), .o(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n144), .c(new_n150), .d(new_n153), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  norp02aa1n02x5               g063(.a(new_n158), .b(new_n144), .o1(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  oao003aa1n02x5               g065(.a(\a[14] ), .b(\b[13] ), .c(new_n155), .carry(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n160), .c(new_n150), .d(new_n153), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norp02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n168), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(\s[16] ));
  nano23aa1n02x4               g076(.a(new_n164), .b(new_n166), .c(new_n167), .d(new_n165), .out0(new_n172));
  nona22aa1n02x4               g077(.a(new_n172), .b(new_n158), .c(new_n144), .out0(new_n173));
  norp02aa1n02x5               g078(.a(new_n173), .b(new_n148), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n129), .c(new_n114), .d(new_n122), .o1(new_n175));
  nanp03aa1n02x5               g080(.a(new_n146), .b(new_n145), .c(new_n147), .o1(new_n176));
  aoi012aa1n02x5               g081(.a(new_n173), .b(new_n176), .c(new_n151), .o1(new_n177));
  aoi012aa1n02x5               g082(.a(new_n166), .b(new_n164), .c(new_n167), .o1(new_n178));
  oaib12aa1n02x5               g083(.a(new_n178), .b(new_n161), .c(new_n172), .out0(new_n179));
  norp02aa1n02x5               g084(.a(new_n177), .b(new_n179), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n175), .b(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g087(.clk(\a[18] ), .clkout(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[17] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\b[16] ), .clkout(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d04x5               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n185), .b(new_n184), .o1(new_n189));
  oaoi03aa1n02x5               g094(.a(\a[18] ), .b(\b[17] ), .c(new_n189), .o1(new_n190));
  norp02aa1n02x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n192), .b(new_n191), .out0(new_n193));
  aoai13aa1n02x5               g098(.a(new_n193), .b(new_n190), .c(new_n181), .d(new_n188), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(new_n193), .b(new_n190), .c(new_n181), .d(new_n188), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  nona22aa1n02x4               g105(.a(new_n194), .b(new_n200), .c(new_n191), .out0(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n191), .clkout(new_n202));
  aobi12aa1n02x5               g107(.a(new_n200), .b(new_n194), .c(new_n202), .out0(new_n203));
  norb02aa1n02x5               g108(.a(new_n201), .b(new_n203), .out0(\s[20] ));
  nano23aa1n02x4               g109(.a(new_n191), .b(new_n198), .c(new_n199), .d(new_n192), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n188), .b(new_n205), .o1(new_n206));
  oai022aa1n02x5               g111(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n207));
  oaib12aa1n02x5               g112(.a(new_n207), .b(new_n183), .c(\b[17] ), .out0(new_n208));
  nona23aa1n02x4               g113(.a(new_n199), .b(new_n192), .c(new_n191), .d(new_n198), .out0(new_n209));
  aoi012aa1n02x5               g114(.a(new_n198), .b(new_n191), .c(new_n199), .o1(new_n210));
  oai012aa1n02x5               g115(.a(new_n210), .b(new_n209), .c(new_n208), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n206), .c(new_n175), .d(new_n180), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[21] ), .b(\b[20] ), .out0(new_n216));
  xorc02aa1n02x5               g121(.a(\a[22] ), .b(\b[21] ), .out0(new_n217));
  aoi112aa1n02x5               g122(.a(new_n215), .b(new_n217), .c(new_n213), .d(new_n216), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n217), .b(new_n215), .c(new_n213), .d(new_n216), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g125(.clk(\a[21] ), .clkout(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(\a[22] ), .clkout(new_n222));
  xroi22aa1d04x5               g127(.a(new_n221), .b(\b[20] ), .c(new_n222), .d(\b[21] ), .out0(new_n223));
  nanp03aa1n02x5               g128(.a(new_n223), .b(new_n188), .c(new_n205), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\b[21] ), .clkout(new_n225));
  oaoi03aa1n02x5               g130(.a(new_n222), .b(new_n225), .c(new_n215), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n226), .clkout(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n211), .c(new_n223), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n224), .c(new_n175), .d(new_n180), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xorc02aa1n02x5               g137(.a(\a[24] ), .b(\b[23] ), .out0(new_n233));
  aoi112aa1n02x5               g138(.a(new_n231), .b(new_n233), .c(new_n229), .d(new_n232), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n233), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(\s[24] ));
  and002aa1n02x5               g141(.a(new_n233), .b(new_n232), .o(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  nano32aa1n02x4               g143(.a(new_n238), .b(new_n223), .c(new_n188), .d(new_n205), .out0(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n210), .clkout(new_n240));
  aoai13aa1n02x5               g145(.a(new_n223), .b(new_n240), .c(new_n205), .d(new_n190), .o1(new_n241));
  norp02aa1n02x5               g146(.a(\b[23] ), .b(\a[24] ), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  aoi012aa1n02x5               g148(.a(new_n242), .b(new_n231), .c(new_n243), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n238), .c(new_n241), .d(new_n226), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[25] ), .b(\b[24] ), .out0(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n245), .c(new_n181), .d(new_n239), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n246), .b(new_n245), .c(new_n181), .d(new_n239), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(\s[25] ));
  norp02aa1n02x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  nona22aa1n02x4               g156(.a(new_n247), .b(new_n251), .c(new_n250), .out0(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n250), .clkout(new_n253));
  aobi12aa1n02x5               g158(.a(new_n251), .b(new_n247), .c(new_n253), .out0(new_n254));
  norb02aa1n02x5               g159(.a(new_n252), .b(new_n254), .out0(\s[26] ));
  nanp02aa1n02x5               g160(.a(new_n123), .b(new_n130), .o1(new_n256));
  oabi12aa1n02x5               g161(.a(new_n179), .b(new_n153), .c(new_n173), .out0(new_n257));
  and002aa1n02x5               g162(.a(new_n251), .b(new_n246), .o(new_n258));
  nano22aa1n02x4               g163(.a(new_n224), .b(new_n237), .c(new_n258), .out0(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n257), .c(new_n256), .d(new_n174), .o1(new_n260));
  oao003aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n261));
  aobi12aa1n02x5               g166(.a(new_n261), .b(new_n245), .c(new_n258), .out0(new_n262));
  norp02aa1n02x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  nanp02aa1n02x5               g168(.a(\b[26] ), .b(\a[27] ), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n264), .b(new_n263), .out0(new_n265));
  xnbna2aa1n03x5               g170(.a(new_n265), .b(new_n262), .c(new_n260), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n263), .clkout(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[27] ), .b(\a[28] ), .out0(new_n268));
  aobi12aa1n02x5               g173(.a(new_n259), .b(new_n175), .c(new_n180), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n237), .b(new_n227), .c(new_n211), .d(new_n223), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n258), .clkout(new_n271));
  aoai13aa1n02x5               g176(.a(new_n261), .b(new_n271), .c(new_n270), .d(new_n244), .o1(new_n272));
  oai012aa1n02x5               g177(.a(new_n264), .b(new_n272), .c(new_n269), .o1(new_n273));
  aoi012aa1n02x5               g178(.a(new_n268), .b(new_n273), .c(new_n267), .o1(new_n274));
  aobi12aa1n02x5               g179(.a(new_n264), .b(new_n262), .c(new_n260), .out0(new_n275));
  nano22aa1n02x4               g180(.a(new_n275), .b(new_n267), .c(new_n268), .out0(new_n276));
  norp02aa1n02x5               g181(.a(new_n274), .b(new_n276), .o1(\s[28] ));
  nano22aa1n02x4               g182(.a(new_n268), .b(new_n267), .c(new_n264), .out0(new_n278));
  oai012aa1n02x5               g183(.a(new_n278), .b(new_n272), .c(new_n269), .o1(new_n279));
  oao003aa1n02x5               g184(.a(\a[28] ), .b(\b[27] ), .c(new_n267), .carry(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[28] ), .b(\a[29] ), .out0(new_n281));
  aoi012aa1n02x5               g186(.a(new_n281), .b(new_n279), .c(new_n280), .o1(new_n282));
  aobi12aa1n02x5               g187(.a(new_n278), .b(new_n262), .c(new_n260), .out0(new_n283));
  nano22aa1n02x4               g188(.a(new_n283), .b(new_n280), .c(new_n281), .out0(new_n284));
  norp02aa1n02x5               g189(.a(new_n282), .b(new_n284), .o1(\s[29] ));
  xorb03aa1n02x5               g190(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g191(.a(new_n265), .b(new_n281), .c(new_n268), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n287), .b(new_n272), .c(new_n269), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .c(new_n280), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[29] ), .b(\a[30] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x5               g196(.a(new_n287), .b(new_n262), .c(new_n260), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n02x5               g198(.a(new_n291), .b(new_n293), .o1(\s[30] ));
  norb03aa1n02x5               g199(.a(new_n278), .b(new_n290), .c(new_n281), .out0(new_n295));
  aobi12aa1n02x5               g200(.a(new_n295), .b(new_n262), .c(new_n260), .out0(new_n296));
  oao003aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .c(new_n289), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[30] ), .b(\a[31] ), .out0(new_n298));
  nano22aa1n02x4               g203(.a(new_n296), .b(new_n297), .c(new_n298), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n295), .b(new_n272), .c(new_n269), .o1(new_n300));
  aoi012aa1n02x5               g205(.a(new_n298), .b(new_n300), .c(new_n297), .o1(new_n301));
  norp02aa1n02x5               g206(.a(new_n301), .b(new_n299), .o1(\s[31] ));
  xnbna2aa1n03x5               g207(.a(new_n111), .b(new_n105), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g208(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n304));
  xorb03aa1n02x5               g209(.a(new_n304), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g210(.a(new_n114), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oab012aa1n02x4               g211(.a(new_n114), .b(\a[5] ), .c(\b[4] ), .out0(new_n307));
  aoi012aa1n02x5               g212(.a(new_n307), .b(\a[5] ), .c(\b[4] ), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g214(.a(new_n120), .b(new_n307), .c(\b[4] ), .d(\a[5] ), .o1(new_n310));
  oaib12aa1n02x5               g215(.a(new_n310), .b(new_n124), .c(\a[6] ), .out0(new_n311));
  xnbna2aa1n03x5               g216(.a(new_n311), .b(new_n127), .c(new_n118), .out0(\s[7] ));
  oaoi03aa1n02x5               g217(.a(\a[7] ), .b(\b[6] ), .c(new_n311), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g219(.a(new_n131), .b(new_n123), .c(new_n130), .out0(\s[9] ));
endmodule


