// Benchmark "adder" written by ABC on Thu Jul 11 11:19:46 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
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
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n301, new_n304, new_n306,
    new_n307, new_n309, new_n310, new_n311;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  aoi012aa1n02x5               g011(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n107));
  oai012aa1n02x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  xnrc02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .out0(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  norp03aa1n02x5               g020(.a(new_n114), .b(new_n115), .c(new_n109), .o1(new_n116));
  oai012aa1n02x5               g021(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n117));
  orn002aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .o(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  oaib12aa1n02x5               g024(.a(new_n117), .b(new_n114), .c(new_n119), .out0(new_n120));
  xorc02aa1n02x5               g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  aoai13aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(new_n108), .d(new_n116), .o1(new_n122));
  xorc02aa1n02x5               g027(.a(\a[10] ), .b(\b[9] ), .out0(new_n123));
  xnbna2aa1n03x5               g028(.a(new_n123), .b(new_n122), .c(new_n97), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g029(.clk(\a[10] ), .clkout(new_n125));
  160nm_ficinv00aa1n08x5       g030(.clk(\b[9] ), .clkout(new_n126));
  norp02aa1n02x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  oai112aa1n02x5               g034(.a(new_n122), .b(new_n97), .c(\b[9] ), .d(\a[10] ), .o1(new_n130));
  oai112aa1n02x5               g035(.a(new_n130), .b(new_n129), .c(new_n126), .d(new_n125), .o1(new_n131));
  oaoi13aa1n02x5               g036(.a(new_n129), .b(new_n130), .c(new_n125), .d(new_n126), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n127), .clkout(new_n134));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n131), .c(new_n134), .out0(\s[12] ));
  nanp02aa1n02x5               g043(.a(new_n108), .b(new_n116), .o1(new_n139));
  oai022aa1n02x5               g044(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n140));
  aboi22aa1n03x5               g045(.a(new_n114), .b(new_n119), .c(new_n111), .d(new_n140), .out0(new_n141));
  nano23aa1n02x4               g046(.a(new_n127), .b(new_n135), .c(new_n136), .d(new_n128), .out0(new_n142));
  nanp03aa1n02x5               g047(.a(new_n142), .b(new_n121), .c(new_n123), .o1(new_n143));
  nona23aa1n02x4               g048(.a(new_n136), .b(new_n128), .c(new_n127), .d(new_n135), .out0(new_n144));
  oai012aa1n02x5               g049(.a(new_n136), .b(new_n135), .c(new_n127), .o1(new_n145));
  oai022aa1n02x5               g050(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n146));
  oaib12aa1n02x5               g051(.a(new_n146), .b(new_n126), .c(\a[10] ), .out0(new_n147));
  oai012aa1n02x5               g052(.a(new_n145), .b(new_n144), .c(new_n147), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n143), .c(new_n139), .d(new_n141), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xorc02aa1n02x5               g060(.a(\a[15] ), .b(\b[14] ), .out0(new_n156));
  norp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nano23aa1n02x4               g063(.a(new_n152), .b(new_n157), .c(new_n158), .d(new_n153), .out0(new_n159));
  oai012aa1n02x5               g064(.a(new_n158), .b(new_n157), .c(new_n152), .o1(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(new_n160), .clkout(new_n161));
  aoai13aa1n02x5               g066(.a(new_n156), .b(new_n161), .c(new_n150), .d(new_n159), .o1(new_n162));
  aoi112aa1n02x5               g067(.a(new_n156), .b(new_n161), .c(new_n150), .d(new_n159), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(\s[15] ));
  orn002aa1n02x5               g069(.a(\a[15] ), .b(\b[14] ), .o(new_n165));
  xnrc02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .out0(new_n166));
  nanp03aa1n02x5               g071(.a(new_n162), .b(new_n165), .c(new_n166), .o1(new_n167));
  aoi012aa1n02x5               g072(.a(new_n166), .b(new_n162), .c(new_n165), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(\s[16] ));
  xorc02aa1n02x5               g074(.a(\a[16] ), .b(\b[15] ), .out0(new_n170));
  nanp03aa1n02x5               g075(.a(new_n159), .b(new_n156), .c(new_n170), .o1(new_n171));
  norp02aa1n02x5               g076(.a(new_n171), .b(new_n143), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n120), .c(new_n108), .d(new_n116), .o1(new_n173));
  xnrc02aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .out0(new_n174));
  nona23aa1n02x4               g079(.a(new_n158), .b(new_n153), .c(new_n152), .d(new_n157), .out0(new_n175));
  norp03aa1n02x5               g080(.a(new_n175), .b(new_n166), .c(new_n174), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n177));
  orn002aa1n02x5               g082(.a(\a[16] ), .b(\b[15] ), .o(new_n178));
  oai013aa1n02x4               g083(.a(new_n178), .b(new_n174), .c(new_n166), .d(new_n160), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n148), .d(new_n176), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n173), .b(new_n180), .o1(new_n181));
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
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n206), .c(new_n173), .d(new_n180), .o1(new_n213));
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
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n224), .c(new_n173), .d(new_n180), .o1(new_n229));
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
  and002aa1n02x5               g160(.a(new_n251), .b(new_n246), .o(new_n256));
  nano22aa1n02x4               g161(.a(new_n224), .b(new_n237), .c(new_n256), .out0(new_n257));
  nanp02aa1n02x5               g162(.a(new_n181), .b(new_n257), .o1(new_n258));
  oao003aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n259));
  aobi12aa1n02x5               g164(.a(new_n259), .b(new_n245), .c(new_n256), .out0(new_n260));
  xorc02aa1n02x5               g165(.a(\a[27] ), .b(\b[26] ), .out0(new_n261));
  xnbna2aa1n03x5               g166(.a(new_n261), .b(new_n260), .c(new_n258), .out0(\s[27] ));
  norp02aa1n02x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  aobi12aa1n02x5               g169(.a(new_n261), .b(new_n260), .c(new_n258), .out0(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[27] ), .b(\a[28] ), .out0(new_n266));
  nano22aa1n02x4               g171(.a(new_n265), .b(new_n264), .c(new_n266), .out0(new_n267));
  aobi12aa1n02x5               g172(.a(new_n257), .b(new_n173), .c(new_n180), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n237), .b(new_n227), .c(new_n211), .d(new_n223), .o1(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n256), .clkout(new_n270));
  aoai13aa1n02x5               g175(.a(new_n259), .b(new_n270), .c(new_n269), .d(new_n244), .o1(new_n271));
  oai012aa1n02x5               g176(.a(new_n261), .b(new_n271), .c(new_n268), .o1(new_n272));
  aoi012aa1n02x5               g177(.a(new_n266), .b(new_n272), .c(new_n264), .o1(new_n273));
  norp02aa1n02x5               g178(.a(new_n273), .b(new_n267), .o1(\s[28] ));
  norb02aa1n02x5               g179(.a(new_n261), .b(new_n266), .out0(new_n275));
  oai012aa1n02x5               g180(.a(new_n275), .b(new_n271), .c(new_n268), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[28] ), .b(\b[27] ), .c(new_n264), .carry(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  aoi012aa1n02x5               g183(.a(new_n278), .b(new_n276), .c(new_n277), .o1(new_n279));
  aobi12aa1n02x5               g184(.a(new_n275), .b(new_n260), .c(new_n258), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n277), .c(new_n278), .out0(new_n281));
  norp02aa1n02x5               g186(.a(new_n279), .b(new_n281), .o1(\s[29] ));
  xorb03aa1n02x5               g187(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g188(.a(new_n261), .b(new_n278), .c(new_n266), .out0(new_n284));
  oai012aa1n02x5               g189(.a(new_n284), .b(new_n271), .c(new_n268), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[29] ), .b(\a[30] ), .out0(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n285), .c(new_n286), .o1(new_n288));
  aobi12aa1n02x5               g193(.a(new_n284), .b(new_n260), .c(new_n258), .out0(new_n289));
  nano22aa1n02x4               g194(.a(new_n289), .b(new_n286), .c(new_n287), .out0(new_n290));
  norp02aa1n02x5               g195(.a(new_n288), .b(new_n290), .o1(\s[30] ));
  xnrc02aa1n02x5               g196(.a(\b[30] ), .b(\a[31] ), .out0(new_n292));
  norb02aa1n02x5               g197(.a(new_n284), .b(new_n287), .out0(new_n293));
  aobi12aa1n02x5               g198(.a(new_n293), .b(new_n260), .c(new_n258), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[30] ), .b(\b[29] ), .c(new_n286), .carry(new_n295));
  nano22aa1n02x4               g200(.a(new_n294), .b(new_n292), .c(new_n295), .out0(new_n296));
  oai012aa1n02x5               g201(.a(new_n293), .b(new_n271), .c(new_n268), .o1(new_n297));
  aoi012aa1n02x5               g202(.a(new_n292), .b(new_n297), .c(new_n295), .o1(new_n298));
  norp02aa1n02x5               g203(.a(new_n298), .b(new_n296), .o1(\s[31] ));
  xnrb03aa1n02x5               g204(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g205(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n301));
  xorb03aa1n02x5               g206(.a(new_n301), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g207(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaib12aa1n02x5               g208(.a(new_n118), .b(new_n115), .c(new_n108), .out0(new_n304));
  xorb03aa1n02x5               g209(.a(new_n304), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g210(.a(new_n109), .b(new_n304), .out0(new_n306));
  oai012aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  norb02aa1n02x5               g213(.a(new_n111), .b(new_n110), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n112), .c(new_n307), .d(new_n113), .o1(new_n310));
  aoi112aa1n02x5               g215(.a(new_n309), .b(new_n112), .c(new_n307), .d(new_n113), .o1(new_n311));
  norb02aa1n02x5               g216(.a(new_n310), .b(new_n311), .out0(\s[8] ));
  xnbna2aa1n03x5               g217(.a(new_n121), .b(new_n139), .c(new_n141), .out0(\s[9] ));
endmodule


